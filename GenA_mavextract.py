#!/usr/bin/env python
# This is a modified (reduced) version of mavextract.py from pymavlink source

from __future__ import print_function

import os
import struct
import numpy as np
import math
#import GenA_main as GA_m

main_path = os.getcwd()

from pymavlink import mavutil
# For testing:
land_target_lat = -35.3627622717389
land_target_lon = 149.165164232254

def older_message(m, lastm):
    '''return true if m is older than lastm by timestamp'''
    atts = {'time_boot_ms' : 1.0e-3,
            'time_unix_usec' : 1.0e-6,
            'time_usec' : 1.0e-6}
    for a in list(atts.keys()):
        if hasattr(m, a):
            mul = atts[a]
            t1 = m.getattr(a) * mul
            t2 = lastm.getattr(a) * mul
            if t2 >= t1 and t2 - t1 < 60:
                return True
    return False

def process(filename):
    '''process one logfile'''
    temp_log_path = main_path + "/log_convert_test.txt"
    log_write_file = open(temp_log_path, 'w')

    temp_gps_lat = []
    temp_gps_lon = []
    temp_time = []
    temp_xterr = []
    temp_alterr = []

    print("Processing %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=False,
                                      robust_parsing=False)


    dirname = os.path.dirname(filename)

    messages = []

    while True:
        m = mlog.recv_match()
        if m is None:
            break

        mtype = m.get_type()
        if mtype in messages:
            if older_message(m, messages[mtype]):
                continue
        #print(mtype)
        if mtype =="GPS":
            log_write_file.write("%i,%f,%f,%f\n" % (m.TimeUS,m.Lat,m.Lng,m.Alt))
            #print("Here")
            temp_gps_lat.append(m.Lat)
            temp_gps_lon.append(m.Lng)
            temp_time.append(m.TimeUS)
            #np.append(temp_gps,([m.TimeUS,m.Lat,m.Lng,m.Alt]))
        elif mtype =="NTUN":
           # print(m)
           #print("Got NTUN message, xtrack error = %f, time = %i" % (m.XT, m.TimeUS))
           temp_alterr.append([m.TimeUS/1000000.0,abs(m.AltErr)])
           temp_xterr.append([m.TimeUS/1000000.0,abs(m.XT)]) # take aboslute value of xtrack error
        else:
            pass


        '''
        if output and m.get_type() != 'BAD_DATA':
            timestamp = getattr(m, '_timestamp', None)
            if not isbin:
                output.write(struct.pack('>Q', timestamp*1.0e6))
            output.write(m.get_msgbuf())
        '''
    log_write_file.close()
    #print(temp_xterr)
    np_xterr = np.array(temp_xterr)
    #print(np_xterr)
    total_xtrack_error = np.trapz(np_xterr[:,1],np_xterr[:,0])
    #print("Total_crosstrack = %f" % total_xtrack_error)

    np_alterr = np.array(temp_alterr)
    total_alt_error = np.trapz(np_alterr[:,1],np_alterr[:,0])


    total_time = (temp_time[-1] - temp_time[0])/1000000.0 #Get total time in seconds
    land_miss_distance = gps_distance(temp_gps_lat[-1], temp_gps_lon[-1], land_target_lat,land_target_lon)

    #land_miss_distance = gps_distance(temp_gps_lat[-1], GA_m.land_target_lat,temp_gps_lon[-1],GA_m.land_target_lon)

    return total_time, total_xtrack_error, total_alt_error, land_miss_distance


# gps_distance function taken from mp_util.py file in MavProxy source
def gps_distance(lat1, lon1, lat2, lon2):
    '''return distance between two points in meters,
    coordinates are in degrees
    thanks to http://www.movable-type.co.uk/scripts/latlong.html'''

    radius_of_earth = 6378100.0 # in meters

    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    lon1 = math.radians(lon1)
    lon2 = math.radians(lon2)
    dLat = lat2 - lat1
    dLon = lon2 - lon1

    a = math.sin(0.5*dLat)**2 + math.sin(0.5*dLon)**2 * math.cos(lat1) * math.cos(lat2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0-a))
    return radius_of_earth * c