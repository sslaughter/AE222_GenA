#!/usr/bin/env python

# Fly ArduPlane in SITL
import math
import os
import random
from shutil import copyfile

parameter_path = os.getcwd() + "/indv_params/"
default_parameter_file = parameter_path + "base_parameter_file.parm"

mission_path = "E:/User_Files/Documents/learning/Grad School/AE222/Final Project/src/ardupilot/Tools/autotest/indv_missions/"
#mission_path = "/cygdrive/e/User_Files/Documents/learning/Grad School/AE222/Final Project/src/ardupilot/Tools/autotest/indv_missions/"
default_mission_file = mission_path + "base_mission.waypoints"

class ArduGen_Individual:

	def __init__(self,params,iter_n,iter_ind,default = False):
		self.name = "%i_%i" % (iter_n,iter_ind)
		self.isdefault = default
		self.log_name = "ArduPlane"
		self.param_set_path = os.getcwd()
		self.parameter_values = {}

		self.miss_path = ""
		self.param_path = ""

		self.path_time = 0.0
		self.mean_path_error = 0.0
		self.mean_alt_error = 0.0
		self.land_error = 0.0

		self.path_fitness = 0.0

		if default == True:
			for key in params.keys():
				self.parameter_values[key] = float(params[key][0])

		else:
			self.parameter_values = params

		#self.parameter_values["ARMING_CHECK"] = 0

		#print(self.parameter_values)
		print("Individual " + self.name)
		#print(self.parameter_values)

		write_parameter_file(self)
		write_waypoint_file(self)

	def get_path_fitness(self,def_vals,weights):
		self.path_fitness = ((self.path_time/def_vals[0])*weights[0] + (self.mean_path_error/def_vals[1])*weights[1] + (self.mean_alt_error/def_vals[2])*weights[2])


def write_parameter_file(self):

	new_param_path = parameter_path + self.name + ".parm"
	copyfile(default_parameter_file,new_param_path)
	self.param_path = new_param_path

	new_param_file = open(new_param_path,'a')
	for param in self.parameter_values.keys():
		new_param_file.write("\n%s\t%s" % (param,self.parameter_values[param]))
	new_param_file.write("\n%s\t%s" % ("ARMING_CHECK",0.000000))
	new_param_file.close()


#Function below adapted from pymavlink MPLoader.save() function
def write_waypoint_file(self):
    '''save waypoints to a file'''
    new_mission_path = mission_path + self.name + ".waypoints"
    self.miss_path = new_mission_path
    print("Mission Path: %s\n" % self.miss_path)

    copyfile(default_mission_file, new_mission_path)

    f = open(new_mission_path,'a')

    #wpn-1
    seq = 10
    current = 0
    frame = 3
    command = 16 #standard waypoint
    param1 = 0.00000000
    param2 = 0.00000000
    param3 = 0.00000000
    param4 = 0.00000000
    x = self.parameter_values["WPn-1 lat"]
    y = self.parameter_values["WPn-1 lon"]
    z = self.parameter_values["WPn-1 alt"]
    autocontinue = 1
    f.write("%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%u" % (
            seq,current, frame, command,
            param1, param2, param3, param4,
            x, y, z, autocontinue))

    f.write("\r\n")
    #wpn-1
    seq = 11
    current = 0
    frame = 3
    command = 21 #land waypoint
    param1 = 0.00000000
    param2 = 0.00000000
    param3 = 0.00000000
    param4 = 0.00000000
    x = self.parameter_values["WPn lat"]
    y = self.parameter_values["WPn lon"]
    z = 0.000
    autocontinue = 1
    f.write("%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%u\n" % (
            seq,current, frame, command,
            param1, param2, param3, param4,
            x, y, z, autocontinue))

    f.close()


    #def write_param_file():
    	#pass

