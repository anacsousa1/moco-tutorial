# -------------------------------------------------------------------------- #
# OpenSim Moco: exampleMocoInverse.py                                        #
# -------------------------------------------------------------------------- #
# Copyright (c) 2020 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Christopher Dembia                                              #
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

# This example shows how to use the MocoInverse tool to exactly prescribe a
# motion and estimate muscle behavior for walking. The first example does not
# rely on electromyography data, while the second example penalizes deviation
# from electromyography data for a subset of muscles.
#
# See the README.txt next to this file for more information.

import os
os.add_dll_directory("C:/OpenSim 4.4/bin")
import opensim as osim


def solve_moco_inverse(data_folder_path):

    # Construct the MocoInverse tool.
    inverse = osim.MocoInverse()


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
    model_processor.append(osim.ModOpAddReserves(1.0))
    inverse.setModel(model_processor)

    # Construct a TableProcessor of the coordinate data and pass it to the
    # inverse tool. TableProcessors can be used in the same way as
    # ModelProcessors by appending TableOperators to modify the base table.
    # A TableProcessor with no operators, as we have here, simply returns the
    # base table.
    inverse.setKinematics(osim.TableProcessor(coordinates))

    # Initial time, final time, and mesh interval.
    inverse.set_initial_time(0.81)
    inverse.set_final_time(1.79)
    inverse.set_mesh_interval(0.02)

    # By default, Moco gives an error if the kinematics contains extra columns.
    # Here, we tell Moco to allow (and ignore) those extra columns.
    inverse.set_kinematics_allow_extra_columns(True)

    # Solve the problem and write the solution to a Storage file.
    solution = inverse.solve()
    solution.getMocoSolution().write('Results/example3DWalking_MocoInverse_solution.sto')

    # Generate a PDF with plots for the solution trajectory.
    model = model_processor.process()
    report = osim.report.Report(model,
                                'Results/example3DWalking_MocoInverse_solution.sto',
                                bilateral=True)
    # The PDF is saved to the working directory.
    report.generate()


def solve_moco_inverse_with_emg(data_folder_path):

    subject_walk_armless = os.path.join(data_folder_path, "subject_walk_armless.osim")
    grf_walk = os.path.join(data_folder_path, "grf_walk.xml")
    coordinates = os.path.join(data_folder_path, "coordinates.sto")
    electromyography = os.path.join(data_folder_path, "electromyography.sto")

    # This initial block of code is identical to the code above.
    inverse = osim.MocoInverse()
    model_processor = osim.ModelProcessor(subject_walk_armless)
    model_processor.append(osim.ModOpAddExternalLoads(grf_walk))
    model_processor.append(osim.ModOpIgnoreTendonCompliance())
    model_processor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    model_processor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    model_processor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    model_processor.append(osim.ModOpAddReserves(1.0))
    inverse.setModel(model_processor)
    inverse.setKinematics(osim.TableProcessor(coordinates))
    inverse.set_initial_time(0.81)
    inverse.set_final_time(1.79)
    inverse.set_mesh_interval(0.02)
    inverse.set_kinematics_allow_extra_columns(True)

    study = inverse.initialize()
    problem = study.updProblem()

    # Add electromyography tracking.
    emg_tracking = osim.MocoControlTrackingGoal('emg_tracking')
    emg_tracking.setWeight(50.0)
    # Each column in electromyography.sto is normalized so the maximum value in
    # each column is 1.0.
    controls_ref = osim.TimeSeriesTable(electromyography)
    # Scale the tracked muscle activity based on peak levels from
    # "Gait Analysis: Normal and Pathological Function" by
    # Perry and Burnfield, 2010 (digitized by Carmichael Ong).
    soleus = controls_ref.updDependentColumn('soleus')
    gasmed = controls_ref.updDependentColumn('gastrocnemius')
    tibant = controls_ref.updDependentColumn('tibialis_anterior')
    for t in range(0, controls_ref.getNumRows()):
        soleus[t] = 0.77 * soleus[t]
        gasmed[t] = 0.87 * gasmed[t]
        tibant[t] = 0.37 * tibant[t]
    emg_tracking.setReference(osim.TableProcessor(controls_ref))
    # Associate actuators in the model with columns in electromyography.sto.
    emg_tracking.setReferenceLabel('/forceset/soleus_r', 'soleus')
    emg_tracking.setReferenceLabel('/forceset/gasmed_r', 'gastrocnemius')
    emg_tracking.setReferenceLabel('/forceset/gaslat_r', 'gastrocnemius')
    emg_tracking.setReferenceLabel('/forceset/tibant_r', 'tibialis_anterior')
    problem.addGoal(emg_tracking)

    # Solve the problem and write the solution to a Storage file.
    solution = study.solve()
    solution.write('Results/example3DWalking_MocoInverseWithEMG_solution.sto')

    # Write the reference data in a way that's easy to compare to the solution.
    controls_ref.removeColumn('medial_hamstrings')
    controls_ref.removeColumn('biceps_femoris')
    controls_ref.removeColumn('vastus_lateralis')
    controls_ref.removeColumn('vastus_medius')
    controls_ref.removeColumn('rectus_femoris')
    controls_ref.removeColumn('gluteus_maximus')
    controls_ref.removeColumn('gluteus_medius')
    controls_ref.setColumnLabels(['/forceset/soleus_r', '/forceset/gasmed_r',
                                 '/forceset/tibant_r'])
    controls_ref.appendColumn('/forceset/gaslat_r', gasmed)
    osim.STOFileAdapter.write(controls_ref, 'Results/controls_reference.sto')

    # Generate a report comparing MocoInverse solutions without and with EMG
    # tracking.
    model = model_processor.process()
    output = 'Results/example3DWalking_MocoInverseWithEMG_report.pdf'
    ref_files = [
        'Results/example3DWalking_MocoInverseWithEMG_solution.sto',
        'Results/controls_reference.sto']
    report = osim.report.Report(model,
                                'Results/example3DWalking_MocoInverse_solution.sto',
                                output=output, bilateral=True,
                                ref_files=ref_files)
    # The PDF is saved to the working directory.
    report.generate()


# Deal with folders
#   Create folder Results if it doesn't exist
if not os.path.exists("Results") or not os.path.isdir("Results"):
    print("Create folder Results")
    os.makedirs("Results", exist_ok=True)


#   Look for Data in the parent directory
current_directory = os.getcwd()
parent_directory = os.path.dirname(current_directory)
data_folder_path = os.path.join(parent_directory, "Data")


# This problem solves in about 5 minutes.
#solve_moco_inverse(data_folder_path)


# This problem penalizes the deviation from electromyography data for a
# subset of muscles, and solves in about 30 minutes.
solve_moco_inverse_with_emg(data_folder_path)
