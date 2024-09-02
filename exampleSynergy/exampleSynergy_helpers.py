import os
import opensim as osim
import numpy as np
import matplotlib.pyplot as plt

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

    add_coordinate_actuator(model, 'hip_flexion_r', 150)
    add_coordinate_actuator(model, 'knee_angle_r', 300)
    add_coordinate_actuator(model, 'ankle_angle_r', 150)

    model.printToXML("ElaboratedData/torque_driven_model.osim")
    return model



def get_muscle_driven_model():
    current_directory = os.getcwd()
    parent_directory = os.path.dirname(current_directory)
    data_folder_path = os.path.join(parent_directory, "Data")
    link_to_model = os.path.join(data_folder_path, generic_link)

    model = osim.Model(link_to_model)
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


    model.printToXML("ElaboratedData/muscle_driven_model.osim")
    return model