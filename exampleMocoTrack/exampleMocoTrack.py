# -------------------------------------------------------------------------- #
# OpenSim Moco: exampleMocoTrack.py                                          #
# -------------------------------------------------------------------------- #
# Copyright (c) 2019 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Nicholas Bianco                                                 #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License") you may     #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0          #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #

# This example features two different tracking problems solved using the
# MocoTrack tool. 
#  - The first problem demonstrates the basic usage of the tool interface
#    to solve a torque-driven marker tracking problem. 
#  - The second problem shows how to customize a muscle-driven state tracking 
#    problem using more advanced features of the tool interface.
# 
# See the README.txt next to this file for more information.

import os
import opensim as osim


def torque_driven_marker_tracking(data_folder_path):
    # Create and name an instance of the MocoTrack tool.
    track = osim.MocoTrack()
    track.setName("torque_driven_marker_tracking")

    subject_walk_armless = os.path.join(data_folder_path, "subject_walk_armless.osim")
    grf_walk = os.path.join(data_folder_path, "grf_walk.xml")
    marker_trajectories = os.path.join(data_folder_path, "marker_trajectories.trc")

    # Construct a ModelProcessor and add it to the tool. ModelProcessors
    # accept a base model and allow you to easily modify the model by appending
    # ModelOperators. Operations are performed in the order that they are
    # appended to the model.
    # Create the base Model by passing in the model file.
    model_processor = osim.ModelProcessor(subject_walk_armless)
    # Add ground reaction external loads in lieu of a ground-contact model.
    model_processor.append(osim.ModOpAddExternalLoads(grf_walk))
    # Remove all the muscles in the model's ForceSet.
    model_processor.append(osim.ModOpRemoveMuscles())
    # Add CoordinateActuators to the model degrees-of-freedom. This ignores the 
    # pelvis coordinates which already have residual CoordinateActuators.
    model_processor.append(osim.ModOpAddReserves(250))
    track.setModel(model_processor)

    # Use this convenience function to set the MocoTrack markers reference
    # directly from a TRC file. By default, the markers data is filtered at
    # 6 Hz and if in millimeters, converted to meters.
    track.setMarkersReferenceFromTRC(marker_trajectories)

    # There is marker data in the 'marker_trajectories.trc' associated with
    # model markers that no longer exists (i.e. markers on the arms). Set this
    # flag to avoid an exception from being thrown.
    track.set_allow_unused_references(True)

    # Increase the global marker tracking weight, which is the weight
    # associated with the internal MocoMarkerTrackingCost term.
    track.set_markers_global_tracking_weight(10)

    # Increase the tracking weights for individual markers in the data set 
    # placed on bony landmarks compared to markers located on soft tissue.
    marker_weights = osim.MocoWeightSet()
    marker_weights.cloneAndAppend(osim.MocoWeight("R.ASIS", 20))
    marker_weights.cloneAndAppend(osim.MocoWeight("L.ASIS", 20))
    marker_weights.cloneAndAppend(osim.MocoWeight("R.PSIS", 20))
    marker_weights.cloneAndAppend(osim.MocoWeight("L.PSIS", 20))
    marker_weights.cloneAndAppend(osim.MocoWeight("R.Knee", 10))
    marker_weights.cloneAndAppend(osim.MocoWeight("R.Ankle", 10))
    marker_weights.cloneAndAppend(osim.MocoWeight("R.Heel", 10))
    marker_weights.cloneAndAppend(osim.MocoWeight("R.MT5", 5))
    marker_weights.cloneAndAppend(osim.MocoWeight("R.Toe", 2))
    marker_weights.cloneAndAppend(osim.MocoWeight("L.Knee", 10))
    marker_weights.cloneAndAppend(osim.MocoWeight("L.Ankle", 10))
    marker_weights.cloneAndAppend(osim.MocoWeight("L.Heel", 10))
    marker_weights.cloneAndAppend(osim.MocoWeight("L.MT5", 5))
    marker_weights.cloneAndAppend(osim.MocoWeight("L.Toe", 2))
    track.set_markers_weight_set(marker_weights)

    # Initial time, final time, and mesh interval. The number of mesh points
    # used to discretize the problem is computed internally using these values.
    track.set_initial_time(0.81)
    track.set_final_time(1.65)
    track.set_mesh_interval(0.05)

    # Solve! Use track.solve() to skip visualizing.
    # solution = track.solveAndVisualize()
    solution = track.solve()
    osim.STOFileAdapter.write(solution.exportToStatesTable(),
                              "Results/example_states.sto")
    osim.STOFileAdapter.write(solution.exportToControlsTable(),
                              "Results/example_controls.sto")


def muscle_driven_state_tracking(data_folder_path):
    # Create and name an instance of the MocoTrack tool.
    track = osim.MocoTrack()
    track.setName("muscle_driven_state_tracking")

    subject_walk_armless = os.path.join(data_folder_path, "subject_walk_armless.osim")
    grf_walk = os.path.join(data_folder_path, "grf_walk.xml")
    coordinates = os.path.join(data_folder_path, "coordinates.sto")

    # Construct a ModelProcessor and set it on the tool. The default
    # muscles in the model are replaced with optimization-friendly
    # DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
    # parameters.
    model_processor = osim.ModelProcessor(subject_walk_armless)
    model_processor.append(osim.ModOpAddExternalLoads(grf_walk))
    model_processor.append(osim.ModOpIgnoreTendonCompliance())
    model_processor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    # Only valid for DeGrooteFregly2016Muscles.
    model_processor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    # Only valid for DeGrooteFregly2016Muscles.
    model_processor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    track.setModel(model_processor)

    # Construct a TableProcessor of the coordinate data and pass it to the 
    # tracking tool. TableProcessors can be used in the same way as
    # ModelProcessors by appending TableOperators to modify the base table.
    # A TableProcessor with no operators, as we have here, simply returns the
    # base table.
    track.setStatesReference(osim.TableProcessor(coordinates))
    track.set_states_global_tracking_weight(10)

    # This setting allows extra data columns contained in the states
    # reference that don't correspond to model coordinates.
    track.set_allow_unused_references(True)

    # Since there is only coordinate position data in the states references,
    # this setting is enabled to fill in the missing coordinate speed data using
    # the derivative of splined position data.
    track.set_track_reference_position_derivatives(True)

    # Initial time, final time, and mesh interval.
    track.set_initial_time(0.81)
    track.set_final_time(1.65)
    track.set_mesh_interval(0.08)

    # Instead of calling solve(), call initialize() to receive a pre-configured
    # MocoStudy object based on the settings above. Use this to customize the
    # problem beyond the MocoTrack interface.
    study = track.initialize()

    # Get a reference to the MocoControlCost that is added to every MocoTrack
    # problem by default.
    problem = study.updProblem()
    effort = osim.MocoControlGoal.safeDownCast(problem.updGoal("control_effort"))

    # Put a large weight on the pelvis CoordinateActuators, which act as the
    # residual, or 'hand-of-god', forces which we would like to keep as small
    # as possible.
    model = model_processor.process()
    model.initSystem()
    force_set = model.getForceSet()
    for i in range(force_set.getSize()):
        force_path = force_set.get(i).getAbsolutePathString()
        if 'pelvis' in str(force_path):
            effort.setWeightForControl(force_path, 10)

    # Solve and visualize.
    solution = study.solve()
    study.visualize(solution)


# Deal with folders
#   Create folder Results if it doesn't exist
if not os.path.exists("Results") or not os.path.isdir("Results"):
    print("Create folder Results")
    os.makedirs("Results", exist_ok=True)


#   Look for Data in the parent directory
current_directory = os.getcwd()
parent_directory = os.path.dirname(current_directory)
data_folder_path = os.path.join(parent_directory, "Data")

# Solve the torque-driven marker tracking problem.
# This problem takes a few minutes to solve.
# torque_driven_marker_tracking(data_folder_path)

# Solve the muscle-driven state tracking problem.
# This problem could take an hour or more to solve, depending on the number of
# processor cores available for parallelization. With 12 cores, it takes around
# 25 minutes.
muscle_driven_state_tracking(data_folder_path)
