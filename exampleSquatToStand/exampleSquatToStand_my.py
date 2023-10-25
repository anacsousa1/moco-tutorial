# #Part 0: Load the Moco libraries and pre-configured Models.
# These models are provided for you (i.e., they are not part of Moco).
import os
os.add_dll_directory("C:/OpenSim 4.4/bin")
import opensim as osim
from ExampleSquatToStandHelpers import get_muscle_driven_model, compare_inverse_solutions, get_torque_driven_model
from mocoPlotTrajectory import mocoPlotTrajectory
import numpy as np

torque_driven_model = get_torque_driven_model()
muscle_driven_model = get_muscle_driven_model()

torque_driven_model.printToXML("ElaboratedData/torque_driven_model.osim")
muscle_driven_model.printToXML("ElaboratedData/muscle_driven_model.osim")

# #Part 1: Torque-driven Predictive Problem
# Part 1a: Create a new MocoStudy.
study = osim.MocoStudy()

# Part 1b: Initialize the problem and set the model.
problem = study.updProblem()
problem.setModelAsCopy(torque_driven_model)

# Part 1c: Set bounds on the problem.
#
# problem.setTimeBounds(initial_bounds, final_bounds)
# problem.setStateInfo(path, trajectory_bounds, inital_bounds, final_bounds)
#
# All *_bounds arguments can be set to a range, [lower upper], or to a
# single value (equal lower and upper bounds). Empty brackets, [], indicate
# using default bounds (if they exist). You may set multiple state infos at
# once using setStateInfoPattern():
#
# problem.setStateInfoPattern(pattern, trajectory_bounds, inital_bounds, ...
#       final_bounds)
#
# This function supports regular expressions in the 'pattern' argument;
# use '.*' to match any substring of the state/control path
# For example, the following will set all coordinate value state infos:
#
# problem.setStateInfoPattern('/path/to/states/.*/value', ...)

# Time bounds
problem.setTimeBounds(0.0, 1.0)

# Position bounds: the model should start in a squat and finish
# standing up.
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [np.deg2rad(-110), np.deg2rad(-5)], np.deg2rad(-107),
                     np.deg2rad(-11))
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [np.deg2rad(-110), np.deg2rad(5)], np.deg2rad(-108),
                     np.deg2rad(0))
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [np.deg2rad(-26), np.deg2rad(-1)], np.deg2rad(-26),
                     np.deg2rad(-1))

# Velocity bounds: all model coordinates should start and end at rest.
problem.setStateInfoPattern('/jointset/.*/speed', [], 0.0, 0.0)

# Part 1d: Add a MocoControlCost to the problem.
problem.addGoal(osim.MocoControlGoal('myeffort'))

# Part 1e: Configure the solver.
solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(25)
solver.set_optim_convergence_tolerance(1e-4)
solver.set_optim_constraint_tolerance(1e-4)

if not os.path.isfile('Results/torque_driven_predicted_solution.sto'):
    # Part 1f: Solve! Write the solution to file, and visualize.
    predict_solution = study.solve()
    predict_solution.write('Results/torque_driven_predicted_solution.sto')
    study.visualize(predict_solution)

# # Part 2: Torque-driven Tracking Problem
# Part 2a: Construct a tracking reference TimeSeriesTable using filtered
# data from the previous solution. Use a TableProcessor, which accepts a
# base table and allows appending operations to modify the table.
torque_driven_reference = osim.TableProcessor("Results/torque_driven_predicted_solution.sto")
torque_driven_reference.append((osim.TabOpLowPassFilter(6)))

# Part 2b: Add a MocoStateTrackingCost to the problem using the states
# from the predictive problem (via the TableProcessor we just created).
# Enable the setAllowUnusedReferences() setting to ignore the controls in
# the predictive solution.
tracking = osim.MocoStateTrackingGoal()
tracking.setName('my_tracking_torque')
tracking.setReference(torque_driven_reference)
tracking.setAllowUnusedReferences(True)

problem.addGoal(tracking)

# Part 2c: Reduce the control cost weight so it now acts as a regularization
# term.
problem.updGoal('myeffort').setWeight(0.001)

# Part 2d: Set the initial guess using the predictive problem solution.
# Tighten convergence tolerance to ensure smooth controls.
solver.setGuessFile('Results/torque_driven_predicted_solution.sto')
solver.set_optim_convergence_tolerance(1e-6)

if not os.path.isfile('Results/torque_driven_tracking_solution.sto'):
    # Part 2e: Solve! Write the solution to file, and visualize.
    tracking_solution = study.solve()
    tracking_solution.write('Results/torque_driven_tracking_solution.sto')
    study.visualize(tracking_solution)

# # Part 3: Compare Predictive and Tracking Solutions
# This is a convenience function provided for you. See mocoPlotTrajectory.m
mocoPlotTrajectory('Results/torque_driven_predicted_solution.sto', 'Results/torque_driven_tracking_solution.sto',
                   'predict', 'track')

# # Part 4: Muscle-driven Inverse Problem
# Create a MocoInverse tool instance.
inverse = osim.MocoInverse()

# Part 4a: Provide the model via a ModelProcessor. Similar to the TableProcessor,
# you can add operators to modify the base model.
model_processor = osim.ModelProcessor(muscle_driven_model)
model_processor.append(osim.ModOpAddReserves(2))
inverse.setModel(model_processor)

# Part 4b: Set the reference kinematics using the same TableProcessor we used
# in the tracking problem.
inverse.setKinematics(torque_driven_reference)

# Part 4c: Set the time range, mesh interval, and convergence tolerance.
inverse.set_initial_time(0)
inverse.set_final_time(1)
inverse.set_mesh_interval(0.05)
inverse.set_convergence_tolerance(1e-4)
inverse.set_constraint_tolerance(1e-4)

# Allow extra (unused) columns in the kinematics and minimize activations.
inverse.set_kinematics_allow_extra_columns(True)
inverse.set_minimize_sum_squared_activations(True)

# Append additional outputs path for quantities that are calculated
# post-hoc using the inverse problem solution.
inverse.append_output_paths('.*normalized_fiber_length')
inverse.append_output_paths('.*passive_force_multiplier')

# Part 4d: Solve! Write the MocoSolution to file.
inverse_solution = inverse.solve()
solution = inverse_solution.getMocoSolution()
solution.write('Results/muscle_driven_inverse_solution.sto')

# Part 4e: Get the outputs we calculated from the inverse solution.
inverse_outputs = inverse_solution.getOutputs()
sto = osim.STOFileAdapter()
sto.write(inverse_outputs, 'Results/muscle_outputs.sto')

# # Part 5: Muscle-driven Inverse Problem with Passive Assistance
# Part 5a: Create a new muscle-driven model, now adding a SpringGeneralizedForce
# about the knee coordinate.
device = osim.SpringGeneralizedForce('knee_angle_r')
device.setStiffness(50)
device.setRestLength(0)
device.setViscosity(0)
muscle_driven_model.addForce(device)

# Part 5b: Create a ModelProcessor similar to the previous one, using the same
# reserve actuator strength so we can compare muscle activity accurately.
model_processor = osim.ModelProcessor(muscle_driven_model)
model_processor.append(osim.ModOpAddReserves(2))
inverse.setModel(model_processor)

# Part 5c: Solve! Write solution.
inverse_device_solution = inverse.solve()
device_solution = inverse_device_solution.getMocoSolution()
device_solution.write('Results/muscle_driven_inverse_device_solution.sto')

# # Part 6: Compare unassisted and assisted Inverse Problems.
print('Cost without device: ', solution.getObjective())
print('Cost with device: ', device_solution.getObjective())

# This is a convenience function provided for you. See below for the
# implementation.
compare_inverse_solutions(inverse_solution, inverse_device_solution)
