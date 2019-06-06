#!/usr/bin/env python
import math
import os
import numpy as np
import random

import GenA_individuals as indv
import GenA_Test
import GenA_mavextract as GParse

from copy import deepcopy
import time

start_time = time.time()
frame = 'plane-elevrev'
main_path = os.getcwd()
sitl_path = "/cygdrive/e/User_Files/Documents/learning/Grad School/AE222/Final Project/src/ardupilot/build/sitl/bin/arduplane"

log_path = main_path + "/logs/"

individuals = {}
original_param_set = {}
num_parameters = 0

#Genetic Algorithm Parameters
num_individuals = 20
interation_num = 1
num_iterations = 25
num_selected_parents = 5
pct_cross = .4

#Fitness Parameters
land_target_lat = -35.3627622717389
land_target_lon = 149.165164232254
land_target_alt = 0.0
indv_fitness = {}

time_weight = .6
xterr_weight = .3
alterr_weight = .1

path_fitness_weights = (time_weight, xterr_weight, alterr_weight)

def_time = 0.0
def_xterr = 0.0
def_alterr = 0.0
def_landerr = 0.0


master_land_params = ("WPn-1 lat", "WPn-1 lon","WPn-1 alt","WPn lat","WPn lon", "LAND_FLARE_SEC","TECS_LAND_ARSPD","TECS_LAND_SPDWEIGHT",
				"TECS_LAND_SINK","TECS_LAND_TCONST","TECS_LAND_DAMP","TECS_LAND_PMAX","TECS_LAND_SRC","TECS_LAND_TDAMP","TECS_LAND_IGAIN",
				"TECS_LAND_PDAMP")

# Initialize individual


def get_param_set():

	#Pulls parameters from parameter_space csv file and creates a dictionary with each parameter name as a key
	# and then the values being Default, Lower Bound, Upper Bound, increment

	param_set_path = main_path + "/test_params.txt"
	param_set_file = open(param_set_path, 'r')


	for line in param_set_file:
		param_info = line.split(',')
		original_param_set[param_info[0]] = (param_info[1],param_info[2],param_info[3],param_info[4])

	param_set_file.close()
	num_parameters = len(original_param_set)


def initialize_individuals(parameter_set):

	# Defines the initial individuals, including one that utilizes the default values

	num_params = len(parameter_set)
	
	temp_individual_count = 1
	default_ind = indv.ArduGen_Individual(parameter_set,interation_num,temp_individual_count,True)
	individuals[default_ind.name] = default_ind
	temp_individual_count += 1
	
	for i in range(num_individuals-1):
		temp_param_set = {}
		for key in parameter_set.keys():
			key_lb = float(parameter_set[key][1]) # Parameter lower bound
			key_ub = float(parameter_set[key][2]) # Parameter upper bound
			key_incr = float(parameter_set[key][3]) # Parameter increment size
			key_center = .5*(key_ub-key_lb) + key_lb # Parameter center value
			key_num_incr = int(abs(key_ub - key_lb)/key_incr)-1 # Number of increments in parameter range
			temp_param_set[key] = (random.randint(-key_num_incr/2,key_num_incr/2)*key_incr + key_center) # Get random valid value in parameter range

		temp_indv = indv.ArduGen_Individual(temp_param_set,interation_num,temp_individual_count,False)
		individuals[temp_indv.name] = temp_indv
		temp_individual_count +=1
	
	print(len(individuals))

	#print(individuals[default_ind.name].parameter_values)

	#temp_param_set = {}



	#pass

	#for i in len(parameer_set):

def check_bounds():

	bound_errors = 0
	for individual in individuals.values():
		for key in individual.parameter_values.keys():
			key_lb = float(original_param_set[key][1]) # Parameter lower bound
			key_ub = float(original_param_set[key][2]) # Parameter upper bound
			key_incr = float(original_param_set[key][3]) # Parameter increment size
			key_center = .5*(key_ub-key_lb) + key_lb # Parameter center value
			key_num_incr = int(abs(key_ub - key_lb)/key_incr) # Number of increments in parameter range
			if individual.parameter_values[key] < key_lb or individual.parameter_values[key] > key_ub:
				key_def = float(original_param_set[key][0])
				if individual.parameter_values[key] != key_def:
					print("Parameter bound error\n")
					print("Key: %s, LB: %f, UB: %f, Value: %f, Center = %f, Incr = %f, Num_incr = %f" % (key,key_lb,key_ub,individual.parameter_values[key],key_center,key_incr,key_num_incr))
					print("\n")
					bound_errors += 1
				else:
					pass
			else:
				pass
	if bound_errors == 0:
		print("No Bound Errors!")
	else:
		pass

def test_individual_set():
	#Handle logs; plan is to let SITL repeatedly create log ..01.BIN, moving the log after each iteration to make room for the new one

	#Check if log ..01 exists, if it does rename it to avoid issues with log creation
	def_log_file = log_path + "00000001.BIN"
	if os.path.isfile(def_log_file):
		os.rename(def_log_file,def_log_file+"_old")
	else:
		pass

	#Create new folder for each individual set (generation)
	indv_set_folder = log_path + str(interation_num)
	if not os.path.isdir(indv_set_folder):
		os.mkdir(indv_set_folder)
	else:
		pass

	#Iterate over each individual
	for ind in individuals.values():
		print("Testing individual: %s" % ind.name)
		#define parameter file (maybe do in individual class)

		#initialize Auotest object
		test_indv = GenA_Test.AutoTestPlane(sitl_path,ind)

		#Run test script on Autotest object
		test_indv.autotest()
		
		#Handle moving log to make room for next log
		indv_log = log_path + "00000001.BIN"
		new_indv_log = log_path + str(interation_num) + "/" + ind.name + ".BIN"
		os.rename(indv_log,new_indv_log)

		ind.path_time, ind.mean_path_error, ind.mean_alt_error, ind.land_error = GParse.process(new_indv_log)
		indv_fitness[ind.name] = (ind.path_time, ind.mean_path_error, ind.mean_alt_error, ind.land_error)

	print(indv_fitness)
	if interation_num == 1:
		global def_time, def_xterr, def_alterr, def_landerr
		def_time = indv_fitness["1_1"][0]
		print("def_time = %f\n" % def_time)
		def_xterr = indv_fitness["1_1"][1]
		def_alterr = indv_fitness["1_1"][2]
		def_landerr = indv_fitness["1_1"][3]
	else:
		pass

path_fitness_params = {}
land_fitness_params = {}




def select_parents(individual_set):

	super_temp_param_set1 = deepcopy(individual_set)
	super_temp_param_set2 = deepcopy(individual_set)

	print("Selectin Parents\n")
	temp_path_fitness_list = {}
	temp_land_fitness_list = {}
	for individual in super_temp_param_set1.values():
		temp_path_fitness_list[individual.name] = individual.path_fitness
	for individual in super_temp_param_set2.values():
		temp_land_fitness_list[individual.name] = individual.land_error

	# Rank individuals by path fitness score
	ranked_path_fitness = sorted(temp_path_fitness_list.values())
	#print(ranked_path_fitness)
	#Take top 5 scores
	top_path_ranks = ranked_path_fitness[0:num_selected_parents]

	top_path_individual_names = {}

	#Find corresponding individuals to top 5 scores
	for fitness_value in top_path_ranks:
		for key,value in temp_path_fitness_list.items():
			if value == fitness_value:
				top_path_individual_names[key] = value
			else:
				pass

	print(top_path_individual_names)
	# Rank individuals by land fitness score
	ranked_land_fitness = sorted(temp_land_fitness_list.values())
	#print(ranked_land_fitness)
	#Take top 5 scores
	top_land_ranks = ranked_land_fitness[0:num_selected_parents]

	top_land_individual_names = {}

	#Find corresponding individuals to top 5 scores
	for fitness_value in top_land_ranks:
		for key,value in temp_land_fitness_list.items():
			if value == fitness_value:
				top_land_individual_names[key] = value
			else:
				pass
	# We know the individual names, now we make a dict with the actual individuals

	top_path_individuals = {}
	for key in super_temp_param_set1.keys():
		for top_key in top_path_individual_names.keys():
			if key == top_key:
				top_path_individuals[key] = super_temp_param_set1[key]
			else:
				pass

	top_land_individuals = {}
	for key in super_temp_param_set2.keys():
		for top_key in top_land_individual_names.keys():
			if key == top_key:
				top_land_individuals[key] = super_temp_param_set2[key]
			else:
				pass


	return top_path_individuals, top_land_individuals

def small_mutate_all(individual_set_parents,base_params,p_type):
	if p_type == "path":
		mstr = "_mut-p"
	else:
		mstr = "_mut-l"
	'''
	for par in individual_set_parents.keys():
		print(individual_set_parents[par].name)
		print(individual_set_parents[par].parameter_values)
	'''

	mutate_range = .10 # Allow up to 10% change on standard mutate
	for ind in individual_set_parents.values():
		temp_individual = deepcopy(ind)
		temp_individual.name = ind.name + mstr
		temp_params = ind.parameter_values
		for param in temp_params.keys():
			param_lb = float(base_params[param][1]) # Parameter lower bound
			param_ub = float(base_params[param][2]) # Parameter upper bound
			param_incr = float(base_params[param][3]) # Parameter increment size
			param_num_incr = int(abs(param_ub - param_lb)/param_incr) # Number of increments in parameter range
			inc_range = int(mutate_range*param_num_incr)
			temp_int = random.randint(-inc_range,inc_range)
			temp_value = temp_params[param] + temp_int*param_incr
			if temp_value > param_lb and temp_value < param_ub:
				temp_params[param] = temp_value
			else:
				print("Mutate out of bounds, reversing this mutation")
				temp_value = temp_params[param] - temp_int*param_incr
				temp_params[param] = temp_value
		individual_set_parents[temp_individual.name] = temp_individual

	print("Done Mutating")
	'''
	for par in individual_set_parents.keys():
		print(individual_set_parents[par].name)
		print(individual_set_parents[par].parameter_values)
	'''

	return individual_set_parents

def mate_parents(path_individual_set, land_individual_set):
	# Want to make num_individuals out of 2 * num_selected_parents
	# Should have just passed parameter set through this rather than all the individuals
	num_new_indv = 1
	new_indv_list = {}
	while num_new_indv <= num_individuals:
		path_parent1_name = random.choice(list(path_individual_set.keys()))
		print("Paren1 name = %s" % path_parent1_name)
		path_parent2_name = random.choice(list(path_individual_set.keys()))
		print("Parent2 name = %s" % path_parent2_name)
		if path_parent2_name == path_parent1_name:
			while path_parent2_name == path_parent1_name:
				path_parent2_name = random.choice(list(path_individual_set.keys()))
		else:
			pass

		land_parent1_name = random.choice(list(land_individual_set.keys()))
		land_parent2_name = random.choice(list(land_individual_set.keys()))
		if land_parent2_name == land_parent1_name:
			while land_parent2_name == land_parent1_name:
				land_parent2_name = random.choice(list(land_individual_set.keys()))
		else:
			pass

		path_parent1 = path_individual_set[path_parent1_name]
		path_parent2 = path_individual_set[path_parent2_name]

		land_parent1 = land_individual_set[land_parent1_name]
		land_parent2 = land_individual_set[land_parent2_name]

		parent1 = get_combined_parameters(path_parent1,land_parent1)
		parent2 = get_combined_parameters(path_parent2,land_parent2)
		'''
		print('Parent 1: \n')
		print(parent1.parameter_values)

		print('Parent 2: \n')
		print(parent2.parameter_values)
		'''
		temp_indv = crossover(parent1,parent2)
		new_indv = indv.ArduGen_Individual(temp_indv.parameter_values,interation_num,num_new_indv,False)
		new_indv_list[new_indv.name] = new_indv
		num_new_indv += 1

	return new_indv_list

		#temp_new_indv = crossover(parent1,parent2)




def get_combined_parameters(path_param_in, land_param_in):
	#path_fitness_params = base_params
	#land_fitness_params = base_params

	new_param_in = path_param_in

	path_param_set = path_param_in.parameter_values
	land_param_set = land_param_in.parameter_values


	for k in path_param_set:
		if k in master_land_params:
			path_param_set[k] = land_param_set[k]
		else:
			pass

	path_param_in.parameter_values = path_param_set
	return path_param_in

def get_new_individual_set(individual_set,base_params):

	path_parents, land_parents = select_parents(individual_set)
	print("Path parents: \n")
	print(path_parents)

	path_parents = small_mutate_all(path_parents,base_params,"path")
	print("After_mutate: \n")
	print(path_parents.keys())
	land_parents = small_mutate_all(land_parents,base_params,"land")
	'''
	for par in path_parents.keys():
		print(path_parents[par].name)
		print(path_parents[par].parameter_values)

	print("\n\n Landing values\n")

	for par in land_parents.keys():
		print(land_parents[par].name)
		print(land_parents[par].parameter_values)
	'''
	global individuals
	individuals = mate_parents(path_parents,land_parents)

	#path_individuals = mate_parents(path_parents)
	#land_individuals = mate_parents(land_parents)

def crossover(parent1,parent2):
	num_crossover = int(pct_cross*num_parameters)
	crossed_parameters = []

	# Currently this does not force a crossover of land parameters

	for i in range(num_crossover):
		cross_param = random.choice(list(original_param_set.keys()))
		if cross_param not in crossed_parameters:
			parent1.parameter_values[cross_param] = parent2.parameter_values[cross_param]
		else:
			while cross_param in crossed_parameters:
				cross_param = random.choice(list(original_param_set.keys()))

			parent1.parameter_values[cross_param] = parent2.parameter_values[cross_param]

	return parent1

def give_dummy_fitness(ind):
	ind.path_fitness = random.uniform(0.0,2.0)
	ind.land_error = random.uniform(0.0,2.0)



# Get initial parameter set
get_param_set()

#Initialize initial set of individuals
initialize_individuals(original_param_set)

#Check that parameters are within bounds (need to make this part of the parameter value definition)
check_bounds()

'''
for j in range(num_iterations):
	temp_individuals = deepcopy(individuals)
	for individual in temp_individuals.values():
		give_dummy_fitness(individual) 
		#print(individual.name)
	interation_num += 1
	get_new_individual_set(temp_individuals,original_param_set)
	#for individual in individuals.values():
		#print(individual.name)
		#print(individual.parameter_values)
'''


#Run genetic algorithm
for j in range(num_iterations):

	test_individual_set()
	if interation_num == 1:
		def_vals = [def_time,def_xterr,def_alterr,def_landerr]
	
	for individual in individuals.values():
		individual.get_path_fitness(def_vals, path_fitness_weights)

	temp_individuals = deepcopy(individuals)

	interation_num += 1
	if interation_num <= num_iterations:
		get_new_individual_set(temp_individuals,original_param_set)

fitness_path = os.getcwd() + "/fitness_log2.txt"
fit_file = open(fitness_path,'w')

for run in indv_fitness.keys():
	fit_file.write("%s,%f,%f,%f,%f\n" % (run,indv_fitness[run][0],indv_fitness[run][1],indv_fitness[run][2],indv_fitness[run][3]))

fit_file.close()

end_time = time.time()
total_time = (end_time - start_time)/3600.0
print("Time Run = %fhours\n" % total_time)






'''
print (param_set.keys())
print (param_set['TECS_LAND_PDAMP'][0])
'''