#!/usr/bin/env python

# Fly ArduPlane in SITL; this file is a slightly modified version of "arduplane.py" file within Tool/autotest folder of ArduPilot source
from __future__ import print_function
import math
import os

import pexpect
from pymavlink import mavutil

from pysim import util

from common import AutoTest
from common import NotAchievedException

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
HOME = mavutil.location(-35.362938, 149.165085, 585, 354)
WIND = "0,180,0.0"  # speed,direction,variance
default_waypoint_path = os.getcwd() + "base_mission.waypoints"


class AutoTestPlane(AutoTest):
    def __init__(self,
                 binary,
                 indv,
                 valgrind=False,
                 gdb=False,
                 speedup=10,
                 frame=None,
                 params=None,
                 gdbserver=False,
                 **kwargs):
        super(AutoTestPlane, self).__init__(**kwargs)
        self.binary = binary
        self.valgrind = valgrind
        self.gdb = gdb
        self.frame = frame
        self.params = params
        self.gdbserver = gdbserver

        self.indv = indv
        self.indv_name = indv.name

        self.wpn_lat = indv.parameter_values["WPn lat"]
        self.wpn_lon = indv.parameter_values["WPn lon"]

        self.wpn1_lat = indv.parameter_values["WPn-1 lat"]
        self.wpn1_lon = indv.parameter_values["WPn-1 lon"]
        self.wpn1_alt = indv.parameter_values["WPn-1 alt"]

        self.home = "%f,%f,%u,%u" % (HOME.lat,
                                     HOME.lng,
                                     HOME.alt,
                                     HOME.heading)
        self.homeloc = None
        self.speedup = speedup
        self.speedup_default = 10

        self.sitl = None
        self.hasInit = False

        self.log_name = "ArduPlane"

    def init(self):
        if self.frame is None:
            self.frame = 'plane-elevrev'

        #Give path to the parameter file for the individual
        #param_path = self.indv_name + '.parm'
        #param_path = 'indv_params/' + param_path
        #param_path = self.indv.param_path
        print("param path: %s\n" % self.indv.param_path)

        defaults_file = os.path.join(testdir,
                                     self.indv.param_path)

        # Initialize SITL instance
        self.sitl = util.start_SITL(self.binary,
                                    wipe=True,
                                    model=self.frame,
                                    home=self.home,
                                    speedup=self.speedup,
                                    defaults_file=defaults_file,
                                    valgrind=self.valgrind,
                                    gdb=self.gdb,
                                    gdbserver=self.gdbserver)
        self.mavproxy = util.start_MAVProxy_SITL(
            'ArduPlane', options=self.mavproxy_options())
        #self.mavproxy.expect('Telemetry log: (\S+)\r\n')
        #print("got this expect (telem)\n")
        #logfile = self.mavproxy.match.group(1)
        logfile = "not sure"
        self.progress("LOGFILE %s" % logfile)

        buildlog = self.buildlogs_path("ArduPlane-test.tlog")
        self.progress("buildlog=%s" % buildlog)
        if os.path.exists(buildlog):
            os.unlink(buildlog)
        try:
            os.link(logfile, buildlog)
        except Exception:
            pass

        print("Pre-Parameters")
        self.mavproxy.expect('Received [0-9]+ parameters')
        print("Received parameter\n")
        util.expect_setup_callback(self.mavproxy, self.expect_callback)

        self.expect_list_clear()
        self.expect_list_extend([self.sitl, self.mavproxy])

        self.progress("Started simulator")

        # get a mavlink connection going
        connection_string = '127.0.0.1:19550'
        try:
            self.mav = mavutil.mavlink_connection(connection_string,
                                                  robust_parsing=True)
        except Exception as msg:
            self.progress("Failed to start mavlink connection on %s: %s"
                          % (connection_string, msg))
            raise
        print("Finished connection")
        self.mav.message_hooks.append(self.message_hook)
        self.mav.idle_hooks.append(self.idle_hook)
        self.hasInit = True
        self.progress("Ready to start testing!")


    def fly_mission(self, filename, height_accuracy=-1, target_altitude=None):
        """Fly a mission from a file."""
        #filename = "ap1.txt"
        self.progress("Flying mission %s" % filename)
        self.mavproxy.send('wp load %s\n' % filename)
        self.mavproxy.expect('Flight plan received')
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        self.mavproxy.send('switch 1\n')  # auto mode
        self.wait_mode('AUTO')
        self.set_rc(3,1800)
        self.wait_waypoint(1, 11, max_dist=60, timeout=90)
        self.wait_groundspeed(0, 0.5, timeout=60)
        self.mavproxy.expect("Auto disarmed")
        self.progress("Mission OK")

    def autotest(self):
        """Autotest ArduPlane in SITL."""
        if not self.hasInit:
            self.init()

        self.fail_list = []
        try:
            self.progress("Waiting for a heartbeat with mavlink protocol %s"
                          % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            self.progress("Setting up RC parameters")
            self.set_rc_default()
            self.set_rc(3, 1000)
            self.set_rc(8, 1500)
            self.progress("Waiting for GPS fix")
            self.mav.recv_match(condition='VFR_HUD.alt>10', blocking=True)
            self.mav.wait_gps_fix()
            while self.mav.location().alt < 10:
                self.mav.wait_gps_fix()
            self.homeloc = self.mav.location()
            self.progress("Home location: %s" % self.homeloc)

            temp_miss_path = '"' + self.indv.miss_path + '"'

            self.wait_ready_to_arm()
            self.arm_vehicle()
            
            self.run_test("Mission test",
                          lambda: self.fly_mission(
                              temp_miss_path,
                              height_accuracy=10,
                              target_altitude=self.homeloc.alt+100))

        except pexpect.TIMEOUT as e:
            self.progress("Failed with timeout")
            self.fail_list.append("timeout")

        self.close()

        if len(self.fail_list):
            self.progress("FAILED: %s" % self.fail_list)
            return False
        return True

