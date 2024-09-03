import os
import opensim as osim
import numpy as np
import matplotlib.pyplot as plt
from sklearn.decomposition import NMF
import numpy as np

generic_link = 'leg6dof9musc.osim'

def add_coordinate_actuator(model, coord_name, opt_force):
    coord_set = model.updCoordinateSet()
    actu = osim.CoordinateActuator()
    actu.setName('tau_' + coord_name)
    actu.setCoordinate(coord_set.get(coord_name))
    actu.setOptimalForce(opt_force)
    actu.setMinControl(-1)
    actu.setMaxControl(1)
    model.addComponent(actu)


def get_torque_driven_model():
    current_directory = os.getcwd()
    parent_directory = os.path.dirname(current_directory)
    data_folder_path = os.path.join(parent_directory, "Data")
    link_to_model = os.path.join(data_folder_path, generic_link)

    model = osim.Model(link_to_model)

    osim.ModelFactory.replaceJointWithWeldJoint(model, "ground_pelvis")
    model.finalizeConnections()

    model.updForceSet().clearAndDestroy()
    model.initSystem()

    add_coordinate_actuator(model, 'hip_flexion_r', 100)
    add_coordinate_actuator(model, 'knee_angle_r', 100)
    add_coordinate_actuator(model, 'ankle_angle_r', 50)

    model.printToXML("../Data/torque_driven_model.osim")
    return model



def get_muscle_driven_model():
    current_directory = os.getcwd()
    parent_directory = os.path.dirname(current_directory)
    data_folder_path = os.path.join(parent_directory, "Data")
    link_to_model = os.path.join(data_folder_path, generic_link)

    model = osim.Model(link_to_model)

    osim.ModelFactory.replaceJointWithWeldJoint(model, "ground_pelvis")
    model.finalizeConnections()

    osim.DeGrooteFregly2016Muscle().replaceMuscles(model)

    for m in np.arange(model.getMuscles().getSize()):
        musc = model.updMuscles().get(int(m))
        musc.setMinControl(0.0)
        musc.set_ignore_activation_dynamics(False)
        musc.set_ignore_tendon_compliance(False)
        musc.set_max_isometric_force(2.0 * musc.get_max_isometric_force())
        dgf = osim.DeGrooteFregly2016Muscle.safeDownCast(musc)
        dgf.set_active_force_width_scale(1.5)
        dgf.set_tendon_compliance_dynamics_mode('implicit')
        if str(musc.getName()) == 'soleus_r':
            dgf.set_ignore_passive_fiber_force(True)


    model.printToXML("../Data/muscle_driven_model.osim")
    return model


def torque_driven_predictive_problem(dt):

    torque_driven_model = get_torque_driven_model()

    # Moco study
    study = osim.MocoStudy()
    problem = study.updProblem()
    problem.setModelAsCopy(torque_driven_model)

    # Define bounds
    problem.setTimeBounds(0.0, dt)
    problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [np.deg2rad(-55), np.deg2rad(55)], np.deg2rad(0),
                         np.deg2rad(33))
    problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [np.deg2rad(-60), np.deg2rad(0)], np.deg2rad(0),
                         np.deg2rad(-10))
    problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [np.deg2rad(-15), np.deg2rad(15)], np.deg2rad(0),
                         np.deg2rad(0))
    problem.setStateInfoPattern('/jointset/.*/speed', [], 0.0, 0.0)
    problem.addGoal(osim.MocoControlGoal('myeffort'))

    # Define solver
    solver = study.initCasADiSolver()
    solver.set_num_mesh_intervals(25)
    solver.set_optim_convergence_tolerance(1e-4)
    solver.set_optim_constraint_tolerance(1e-2)

    # Solve
    if not os.path.isfile('Results/torque_driven_predicted_solution.sto'):
        predict_solution = study.solve()
        predict_solution.write('Results/torque_driven_predicted_solution.sto')
        study.visualize(predict_solution)

    return study, problem, solver


def torque_driven_tracking_problem(study, problem, solver):

    # Filter data
    torque_driven_reference = osim.TableProcessor("Results/torque_driven_predicted_solution.sto")
    torque_driven_reference.append((osim.TabOpLowPassFilter(6)))

    # Create tracking goal
    tracking = osim.MocoStateTrackingGoal()
    tracking.setName('my_tracking_torque')
    tracking.setReference(torque_driven_reference)
    tracking.setAllowUnusedReferences(True)
    problem.addGoal(tracking)
    problem.updGoal('myeffort').setWeight(0.001)

    # Use predictive solution as a guess
    solver.setGuessFile('Results/torque_driven_predicted_solution.sto')

    # Decrease tolerance
    solver.set_optim_convergence_tolerance(1e-6)

    # Solve
    if not os.path.isfile('Results/torque_driven_tracking_solution.sto'):
        tracking_solution = study.solve()
        tracking_solution.write('Results/torque_driven_tracking_solution.sto')
        study.visualize(tracking_solution)

    return torque_driven_reference


def muscle_driven_inverse_problem(reference, model, dt):

    # Create inverse problem
    inverse = osim.MocoInverse()
    model_processor = osim.ModelProcessor(model)
    model_processor.append(osim.ModOpAddReserves(2))
    inverse.setModel(model_processor)
    inverse.setKinematics(reference)

    # Set boundaries and tolerances
    inverse.set_initial_time(0)
    inverse.set_final_time(dt)
    inverse.set_mesh_interval(0.05)
    inverse.set_convergence_tolerance(1e-4)
    inverse.set_constraint_tolerance(1e-4)
    inverse.set_kinematics_allow_extra_columns(True)
    inverse.set_minimize_sum_squared_activations(True)

    # Append additional outputs path for quantities that are calculated
    # post-hoc using the inverse problem solution.
    inverse.append_output_paths('.*normalized_fiber_length')
    inverse.append_output_paths('.*passive_force_multiplier')

    # Solve
    if not os.path.isfile('Results/muscle_driven_inverse_solution.sto'):
        inverse_solution = inverse.solve()
        solution = inverse_solution.getMocoSolution()
        solution.unseal()
        solution.write('Results/muscle_driven_inverse_solution.sto')


def moco_inverse_synergy(model, reference, dt, numSynergies, prevSolution):

    # Create an instance of the MocoInverse problem
    inverse = osim.MocoInverse()
    model_processor = osim.ModelProcessor(model)
    model_processor.append(osim.ModOpAddReserves(2))  # Add reserves for the inverse problem

    # Set the model and kinematics for the inverse problem
    inverse.setModel(model_processor)
    inverse.setKinematics(reference)

    # Configure the inverse problem settings
    inverse.set_initial_time(0)
    inverse.set_final_time(dt)
    inverse.set_mesh_interval(0.05)
    inverse.set_convergence_tolerance(1e-2)
    inverse.set_constraint_tolerance(1e-2)
    inverse.set_kinematics_allow_extra_columns(True)
    inverse.set_minimize_sum_squared_activations(True)

    # Append additional output paths for post-hoc calculations
    inverse.append_output_paths('.*normalized_fiber_length')
    inverse.append_output_paths('.*passive_force_multiplier')

    # Solve the inverse problem if the solution file does not already exist
    if not os.path.isfile('Results/muscle_driven_inverse_solution_synergy.sto'):
        # Initialize the inverse problem study
        study = inverse.initialize()
        problem = study.updProblem()

        # Adjust control bounds and weights for synergy excitations
        effort = osim.MocoControlGoal.safeDownCast(
            problem.updGoal("excitation_effort"))
        for i in range(numSynergies):
            name_right = (f'/controllerset/synergy_controller_right_leg'
                         f'/synergy_excitation_{i}')
            problem.setInputControlInfo(name_right, [0, 1.0])
            effort.setWeightForControl(name_right, 10)

        # Solve the problem
        solution = study.solve()

        # Generate control values based on the SynergyControllers
        solution.generateControlsFromModelControllers(model)

        # Add multibody states (values and speeds) to the solution for visualization
        coordinate_values = prevSolution.exportToValuesTable()
        coordinate_speeds = prevSolution.exportToSpeedsTable()
        solution.insertStatesTrajectory(coordinate_values)
        solution.insertStatesTrajectory(coordinate_speeds)

        # Write the solution to a file
        solution_file = ('Results/muscle_driven_inverse_solution_synergy.sto')
        solution.write(solution_file)


def muscle_driven_inverse_problem_synergies(model, reference, dt):

    # Define the number of synergies to extract
    numSynergies = 2

    # Load the previously computed Moco inverse solution
    prevSolution = osim.MocoTrajectory('Results/muscle_driven_inverse_solution.sto')

    # Initialize a list to store the names of the right leg muscle control variables
    right_control_names = list()

    # Initialize the model system to ensure it's ready for querying
    model.initSystem()

    # Iterate over all components in the model to find muscle actuators
    for actu in model.getComponentsList():
        # Check if the component is a muscle
        if actu.getConcreteClassName().endswith('Muscle'):
            # Append the path of the muscle actuator to the list
            right_control_names.append(actu.getAbsolutePathString())

    # Export control signals from the previous solution to a table
    controls = prevSolution.exportToControlsTable()

    # Create a TimeSeriesTable with the control signals
    right_controls = osim.TimeSeriesTable(controls.getIndependentColumn())

    # Append control signals to the TimeSeriesTable based on muscle names
    for name in right_control_names:
        right_controls.appendColumn(name, controls.getDependentColumn(name))

    # Initialize NMF with the specified number of synergies
    nmf = NMF(n_components=numSynergies, init='random', random_state=0)

    # Convert the control signals table to a NumPy matrix
    A = right_controls.getMatrix().to_numpy()

    # Perform NMF to extract synergy excitations (W) and synergy vectors (H)
    W = nmf.fit_transform(A)
    H = nmf.components_

    # Normalize the synergy vectors (H) and adjust the excitations (W)
    scaleVec = 0.5*np.ones(H.shape[1])  # Initial scaling vector
    for i in range(numSynergies):
        # Scale the synergy vector to normalize it
        scale_r = np.linalg.norm(scaleVec) / np.linalg.norm(H[i, :])
        H[i, :] *= scale_r
        W[:, i] /= scale_r

    # Create and configure a SynergyController
    right_controller = osim.SynergyController()
    right_controller.setName("synergy_controller_right_leg")

    # Add actuators to the SynergyController based on the muscle names
    for name in right_control_names:
        right_controller.addActuator(
                osim.Muscle.safeDownCast(model.getComponent(name)))

    # Add synergy vectors to the SynergyController
    for i in range(numSynergies):
        synergy_vector = osim.Vector(H.shape[1], 0.0)
        for j in range(H.shape[1]):
            synergy_vector.set(j, H[i, j])
        right_controller.addSynergyVector(synergy_vector)

    # Add the SynergyController to the model and finalize connections
    model.addController(right_controller)
    model.finalizeConnections()
    model.initSystem()

    # Create and solve the inverse problem with synergies
    moco_inverse_synergy(model, reference, dt, numSynergies, prevSolution)



def muscle_driven_inverse_problem_groups(model, reference, dt):
    """
    Solves a muscle-driven inverse problem by grouping muscles into synergies,
    calculating synergies using NNMF (Non-negative Matrix Factorization), and
    configuring a SynergyController to simulate muscle activity in a model.

    Parameters:
    model (osim.Model): The musculoskeletal model.
    reference (str): Path to reference motion data.
    dt (float): Simulation time step.

    Returns:
    None
    """

    # Load the previous Moco inverse solution for muscle controls.
    prevSolution = osim.MocoTrajectory('Results/muscle_driven_inverse_solution.sto')

    # Define muscle groups where multiple muscles work together as synergies.
    # For example, "hamst" includes two hamstring muscles.
    muscle_groups = {
        "hamst": ["/forceset/bifemlh_r", "/forceset/bifemsh_r"],
        "glut": ["/forceset/glut_max2_r"],
        "quads": ["/forceset/psoas_r", "/forceset/rect_fem_r", "/forceset/vas_int_r"],
        "gast": ["/forceset/med_gas_r", "/forceset/soleus_r"],
        "tibialis": ["/forceset/tib_ant_r"],
    }

    # The number of synergies is equal to the number of muscle groups.
    numSynergies = len(muscle_groups)

    # Initialize the model to prepare it for simulation.
    model.initSystem()

    # Create a list to store the names of all right leg muscles in the model.
    right_control_names = []
    for actu in model.getComponentsList():
        if actu.getConcreteClassName().endswith('Muscle'):
            right_control_names.append(actu.getAbsolutePathString())

    # Export the control signals (e.g., muscle excitations) from the previous solution.
    controls = prevSolution.exportToControlsTable()

    # This will hold the combined control data for each muscle group.
    grouped_control_data = []

    # Loop over each muscle group, combine their control data, and store the result.
    for group_name, muscle_list in muscle_groups.items():
        try:
            # Initialize an array of zeros to store the combined control signal for the group.
            first_muscle_control = controls.getDependentColumn(muscle_list[0]).to_numpy()
            group_control = np.zeros(first_muscle_control.shape)

            # Add up the control signals for each muscle in the group.
            for muscle in muscle_list:
                if muscle in right_control_names:
                    # Convert muscle control data to numpy array and add to the group control.
                    muscle_control = controls.getDependentColumn(muscle).to_numpy().astype(float)
                    group_control += muscle_control
                else:
                    print(f"Warning: Muscle '{muscle}' not found in right_control_names.")

            # Average the control signal across muscles in the group.
            group_control /= len(muscle_list)

            # Store the combined control data for this group.
            grouped_control_data.append(group_control)

        except Exception as e:
            print(f"Error processing group '{group_name}': {e}")

    # Ensure we have valid control data to work with.
    if not grouped_control_data:
        print("Error: No muscle controls were aggregated. Check the muscle names in the groups.")
        return  # Exit the function if no data is available

    # Stack the combined control data vertically to prepare it for factorization (NNMF).
    A = np.vstack(grouped_control_data).T  # Transpose so that time is along rows, and groups are along columns.

    # Apply Non-negative Matrix Factorization (NNMF) to the aggregated control data to find synergies.
    nmf = NMF(n_components=numSynergies, init='random', random_state=0)
    W = nmf.fit_transform(A)  # Synergy excitations over time
    H = nmf.components_  # Synergy vectors that combine muscle controls

    # Scale the synergies for normalisation.
    scaleVec = 0.5 * np.ones(H.shape[1])
    for i in range(numSynergies):
        scale_r = np.linalg.norm(scaleVec) / np.linalg.norm(H[i, :])
        H[i, :] *= scale_r
        W[:, i] /= scale_r

    # Create a SynergyController to control the muscles based on synergies.
    right_controller = osim.SynergyController()
    right_controller.setName("synergy_controller_right_leg")

    # Link each muscle in the right leg to the SynergyController.
    for name in right_control_names:
        right_controller.addActuator(osim.Muscle.safeDownCast(model.getComponent(name)))

    # Add each synergy vector to the controller.
    for i in range(numSynergies):
        # Create an empty synergy vector with size equal to the number of muscles.
        synergy_vector = osim.Vector(len(right_control_names), 0.0)
        idx = 0

        # Assign the components of each synergy to the corresponding muscles.
        for group_name, muscle_list in muscle_groups.items():
            for muscle in muscle_list:
                if muscle in right_control_names:
                    # Assign the NNMF value to the corresponding muscle index.
                    synergy_vector.set(right_control_names.index(muscle), H[i, idx])
            idx += 1
        # Add the synergy vector to the controller.
        right_controller.addSynergyVector(synergy_vector)

    # Attach the SynergyController to the model.
    model.addController(right_controller)

    # Finalize the connections in the model and reinitialize the system for simulation.
    model.finalizeConnections()
    model.initSystem()

    # Create and solve the inverse problem with synergies
    moco_inverse_synergy(model, reference, dt, numSynergies, prevSolution)



def plot_predictive_and_tracking():

    sto_predictive = 'Results/torque_driven_predicted_solution.sto'
    sto_tracking = 'Results/torque_driven_tracking_solution.sto'

    # Get trajectories, control names and number of controls
    trajectory_p = osim.MocoTrajectory(sto_predictive)
    trajectory_t = osim.MocoTrajectory(sto_tracking)

    control_names = list(trajectory_p.getControlNames())
    num_controls = len(control_names)
    dim = np.sqrt(num_controls)

    if dim == np.ceil(dim):
        num_rows = int(dim)
        num_cols = int(dim)
    else:
        num_cols = int(np.min([num_controls, 4]))
        num_rows = int(np.floor(num_controls / 4.0))
        if not num_controls % 4 == 0:
            num_rows += 1

    fig, axs = plt.subplots(num_rows, num_cols, figsize=(12, 6 * num_rows), squeeze=False)
    fig.subplots_adjust(hspace=0.3, wspace=0.4)

    for i in range(num_controls):
        row = i // num_cols
        col = i % num_cols
        ax = axs[row, col]

        # Get control values for both trajectories
        y_p = trajectory_p.getControlMat(control_names[i])
        y_t = trajectory_t.getControlMat(control_names[i])
        time_p = trajectory_p.getTimeMat()
        time_t = trajectory_t.getTimeMat()

        # Plot the predictive trajectory
        ax.plot(time_p, y_p, label='Predictive', linewidth=2, color='blue')

        # Plot the tracking trajectory
        ax.plot(time_t, y_t, label='Tracking', linewidth=2, color='orange', linestyle='--')

        # Customize plot appearance
        ax.set_title(control_names[i])
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Activation')
        ax.set_xlim(0, max(time_p[-1], time_t[-1]))
        ax.set_ylim(-1, 1)
        ax.legend()  # Add a legend to distinguish the lines

    plt.show()


def plot_muscles(sto_muscles):

    # sto_muscles = 'Results/muscle_driven_inverse_solution.sto'

    # Get trajectories, control names and number of controls
    trajectory_p = osim.MocoTrajectory(sto_muscles)

    control_names = list(trajectory_p.getControlNames())
    num_controls = len(control_names)
    dim = np.sqrt(num_controls)

    if dim == np.ceil(dim):
        num_rows = int(dim)
        num_cols = int(dim)
    else:
        num_cols = int(np.min([num_controls, 4]))
        num_rows = int(np.floor(num_controls / 4.0))
        if not num_controls % 4 == 0:
            num_rows += 1

    fig, axs = plt.subplots(num_rows, num_cols, figsize=(12, 6 * num_rows), squeeze=False)
    fig.subplots_adjust(hspace=0.3, wspace=0.4)

    for i in range(num_controls):
        row = i // num_cols
        col = i % num_cols
        ax = axs[row, col]

        # Get control values for both trajectories
        y_p = trajectory_p.getControlMat(control_names[i])
        time_p = trajectory_p.getTimeMat()

        # Plot the predictive trajectory
        ax.plot(time_p, y_p, label='Predictive', linewidth=2, color='blue')

        # Customize plot appearance
        ax.set_title(control_names[i])
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Activation')
        ax.set_xlim(0, time_p[-1])
        ax.set_ylim(-1, 1)

    plt.show()