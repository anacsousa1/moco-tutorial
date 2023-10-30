import os
import opensim as osim
import numpy as np
import matplotlib.pyplot as plt

generic_link = 'Data/squatToStand_3dof9musc.osim'


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
    #   Look for Data in the parent directory
    current_directory = os.getcwd()
    parent_directory = os.path.dirname(current_directory)
    data_folder_path = os.path.join(parent_directory, "Data")

    link_to_model = os.path.join(data_folder_path, generic_link)

    # Load the base model.
    model = osim.Model(link_to_model)

    # Remove the muscles in the model.
    model.updForceSet().clearAndDestroy()
    model.initSystem()

    # Add CoordinateActuators to the model degrees-of-freedom.
    add_coordinate_actuator(model, 'hip_flexion_r', 150)
    add_coordinate_actuator(model, 'knee_angle_r', 300)
    add_coordinate_actuator(model, 'ankle_angle_r', 150)

    model.printToXML("ElaboratedData/torque_driven_model.osim")
    return model


def get_muscle_driven_model():
    #   Look for Data in the parent directory
    current_directory = os.getcwd()
    parent_directory = os.path.dirname(current_directory)
    data_folder_path = os.path.join(parent_directory, "Data")

    link_to_model = os.path.join(data_folder_path, generic_link)

    # Load the base model.
    model = osim.Model(link_to_model)
    model.finalizeConnections()

    # Replace the muscles in the model with muscles from DeGroote, Fregly,
    # et al. 2016, "Evaluation of Direct Collocation Optimal Control Problem
    # Formulations for Solving the Muscle Redundancy Problem". These muscles
    # have the same properties as the original muscles but their characteristic
    # curves are optimized for direct collocation (i.e. no discontinuities,
    # twice differentiable, etc).
    osim.DeGrooteFregly2016Muscle().replaceMuscles(model)

    # Make problems easier to solve by strengthening the model and widening the
    # active force-length curve.
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
            # Soleus has a very long tendon, so modeling its tendon as rigid
            # causes the fiber to be unrealistically long and generate
            # excessive passive fiber force.
            dgf.set_ignore_passive_fiber_force(True)

    model.printToXML("ElaboratedData/muscle_driven_model.osim")
    return model


def compare_inverse_solutions(unassisted_solution, assisted_solution):
    unassisted_solution = unassisted_solution.getMocoSolution()
    assisted_solution = assisted_solution.getMocoSolution()
    state_names = unassisted_solution.getStateNames()
    num_states = len(state_names)

    fig = plt.figure()
    dim = 3
    iplot = 1
    for i in np.arange(num_states):
        if 'activation' in str(state_names[i]):
            ax = fig.add_subplot(dim, dim, iplot)
            ax.plot(unassisted_solution.getTimeMat(),
                    unassisted_solution.getStateMat(state_names[i]),
                    color='red', linewidth=3, label='unassisted')
            ax.plot(assisted_solution.getTimeMat(),
                    assisted_solution.getStateMat(state_names[i]),
                    color='blue', linewidth=2.5, label='assisted')
            plot_title = str(state_names[i])
            plot_title.replace('/forceset/', '')
            plot_title.replace('/activation', '')
            ax.set_title(plot_title)
            ax.set_xlabel('time (s)')
            ax.set_ylabel('activation (-)')
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            if iplot == 1:
                ax.legend()
            iplot += 1

    fig.tight_layout()
    plt.show()
    plt.close()
