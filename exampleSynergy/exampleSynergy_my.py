
import os
import opensim as osim
import exampleSynergy_helpers
import numpy as np

# Folders
if not os.path.exists("Results") or not os.path.isdir("Results"):
    print("Create folder Results")
    os.makedirs("Results", exist_ok=True)

dt = 1.0

muscle_model = exampleSynergy_helpers.get_muscle_driven_model()

# Part 1: Torque-driven predictive problem
study, problem, solver = exampleSynergy_helpers.torque_driven_predictive_problem(dt)

# Part 2: Torque-driven tracking problem (i.e. smooth torque-driven solution)
reference = exampleSynergy_helpers.torque_driven_tracking_problem(study, problem, solver)

# Part 2a: Plot comparison
# exampleSynergy_helpers.plot_predictive_and_tracking()

# # Part 3: Muscle-driven Inverse Problem
exampleSynergy_helpers.muscle_driven_inverse_problem(reference, muscle_model, dt)

# Part 3a: Plot muscle activations
# exampleSynergy_helpers.plot_muscles('Results/muscle_driven_inverse_solution.sto')

# # Part 4: Muscle-driven Inverse Problem with Synergies
exampleSynergy_helpers.muscle_driven_inverse_problem_groups(muscle_model, reference, dt)

# Part 4a: Plot muscle activations
exampleSynergy_helpers.plot_muscles('Results/muscle_driven_inverse_solution_synergy.sto')

