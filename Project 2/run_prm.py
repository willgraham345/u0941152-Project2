# -*- coding: utf-8 -*-
"""
Created on Thu Aug 29 12:17:29 2019

@author: tabor
"""
import numpy as np
import matplotlib.pyplot as plotter
from math import pi
from collisions import PolygonEnvironment
import time
import vrepWrapper
from prm import *

num_samples=750
# problem = "vrep"
problem = 'env1.txt'
_STEP_LENGTH_POLYGON = .01
_STEP_LENGTH_VREP = 0.1
_RADIUS = .75

np.random.seed(0)

def create_env(filename):
    environment = PolygonEnvironment()
    environment.read_env('Project 2/' + filename)
    return environment

#load problem
if(problem == "vrep"):
    environment = vrepWrapper.vrepWrapper()
    step_length=0.1
else:
    environment = PolygonEnvironment()
    environment.read_env('Project 2/' + problem)
    step_length=_STEP_LENGTH_POLYGON
print("Running...")
print(problem + " with " + str(num_samples) + " samples and step length " + str(step_length) + " and radius " + str(_RADIUS))


num_dimensions = len(environment.start)
start_time = time.time()
collision_func = environment.test_collisions
local_planner = StraightLinePlanner(step_length, environment.test_collisions)

prm = PRM(num_samples, local_planner, num_dimensions, environment.lims,
                 collision_func, _RADIUS, _STEP_LENGTH_POLYGON)

prm.build_prm()
# env0_0_0 = create_env('env0_0_0.txt')
# env0_25_neg30 = create_env('env0_25_neg30.txt')
# env0_neg90_40 = create_env('env0_neg90_40.txt')
# plan_0_0_0, visited_0_0_0 = prm.query(np.array([-0.1, -0.1]), env0_0_0.goal) # There is an immediate collision with 0,0 as a starting point (took me 4 hours to figure out)
# plan_25_neg30, visited_25_neg30 = prm.query(env0_25_neg30.start, env0_25_neg30.goal)
# plan_neg90_40, visited_neg90_40 = prm.query(env0_neg90_40.start, env0_neg90_40.goal)
env1_90_0_0 = create_env('env1_90_0_0.txt')
env1_180_0_135 = create_env('env1_180_0_135.txt')
env1_225_45_45 = create_env('env1_225_45_45.txt')
plan_env1, visited_env1 = prm.query(environment.start, environment.goal)
plan_env1_90_0_0, visited_env1_90_0_0 = prm.query(env1_90_0_0.start, env1_90_0_0.goal)
plan_env1_180_0_135, visited_env1_180_0_135 = prm.query(env1_180_0_135.start, env1_180_0_135.goal)
plan_env1_225_45_45, visited_env1_225_45_45 = prm.query(env1_225_45_45.start, env1_225_45_45.goal)
#Plotters
# plot_env0 = environment.draw_plan(plan, prm, False, False, False)
# plot_env0_0_0 = env0_0_0.draw_plan(plan_0_0_0, prm, False, False, False)
# plot_env0_25_neg30 = env0_25_neg30.draw_plan(plan_25_neg30, prm, False, False, False)
# plot_env0_neg90_40 = env0_neg90_40.draw_plan(plan_neg90_40, prm, False, False, False)
plot_env1_no_plan = environment.draw_plan(None, prm, False, False, False)
plot_env1 = environment.draw_plan(plan_env1, prm, False, False, False)
plot_env1_90_0_0 = env1_90_0_0.draw_plan(plan_env1_90_0_0, prm, False, False, False)
plot_env1_180_0_135 = env1_180_0_135.draw_plan(plan_env1_180_0_135, prm, False, False, False)
plot_env1_225_45_45 = env1_225_45_45.draw_plan(plan_env1_225_45_45, prm, False, False, False)


if(problem == "vrep"):
    environment.vrepReset()

run_time = time.time() - start_time
print('plan:', plan)
print('run_time =', run_time)



if(problem == "vrep"):
    time.sleep(10)
    environment.vrepStop()
