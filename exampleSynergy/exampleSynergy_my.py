
import os
import opensim as osim
import exampleSynergy_helpers
import numpy as np

# Folders
if not os.path.exists("Results") or not os.path.isdir("Results"):
    print("Create folder Results")
    os.makedirs("Results", exist_ok=True)
if not os.path.exists("ElaboratedData") or not os.path.isdir("ElaboratedData"):
    print("Create folder ElaboratedData")
    os.makedirs("ElaboratedData", exist_ok=True)

# #Part 1: Torque-driven Predictive Problem
torque_driven_model = exampleSynergy_helpers.get_torque_driven_model()

study = osim.MocoStudy()
problem = study.updProblem()
problem.setModelAsCopy(torque_driven_model)
problem.setTimeBounds(0.0, 1.0)
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [np.deg2rad(-25), np.deg2rad(35)], np.deg2rad(-20),
                     np.deg2rad(30))
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [np.deg2rad(-60), np.deg2rad(-50)], np.deg2rad(-55),
                     np.deg2rad(-55))
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [np.deg2rad(-5), np.deg2rad(5)], np.deg2rad(0),
                     np.deg2rad(0))
problem.setStateInfoPattern('/jointset/.*/speed', [], 0.0, 0.0)
problem.addGoal(osim.MocoControlGoal('myeffort'))

solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(25)
solver.set_optim_convergence_tolerance(1e-4)
solver.set_optim_constraint_tolerance(1e-2)


if not os.path.isfile('Results/torque_driven_predicted_solution.sto'):
    predict_solution = study.solve()
    predict_solution.write('Results/torque_driven_predicted_solution.sto')
    study.visualize(predict_solution)


# # # Part 2: Muscle-driven Inverse Problem with Synergy
# muscle_driven_model = exampleSynergy_helpers.get_muscle_driven_model()
# inverse = osim.MocoInverse()

