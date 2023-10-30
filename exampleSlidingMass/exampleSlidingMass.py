# -------------------------------------------------------------------------- #
# OpenSim Moco: exampleSlidingMass.py                                        #
# -------------------------------------------------------------------------- #
# Copyright (c) 2017 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Christopher Dembia                                              #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License"); you may    #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0          #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #

# Import
import os
os.add_dll_directory("C:/OpenSim 4.4/bin")
import opensim as osim

# Create the model
model = osim.Model()
model.setName('sliding_mass')
model.set_gravity(osim.Vec3(9.81, 0, 0))
body = osim.Body('body', 1.0, osim.Vec3(0), osim.Inertia(0))
model.addComponent(body)

# Allows translation along x.
joint = osim.SliderJoint('slider', model.getGround(), body)
coord = joint.updCoordinate()
coord.setName('position')
model.addComponent(joint)

# Create an actuator
actu = osim.CoordinateActuator()
actu.setCoordinate(coord)
actu.setName('actuator')
actu.setOptimalForce(10)
model.addComponent(actu)

# Attach geometry
body.attachGeometry(osim.Sphere(0.05))

model.finalizeConnections()

# Create MocoStudy.
# ================
study = osim.MocoStudy()
study.setName('sliding_mass')

# Define the optimal control problem.
# ===================================
problem = study.updProblem()

# Model (dynamics).
# -----------------
problem.setModel(model)

# Bounds.
# -------
# Initial time must be 0, final time can be within [0, 5].
problem.setTimeBounds(osim.MocoInitialBounds(0.), osim.MocoFinalBounds(0., 5.))

# Initial position must be 0, final position must be 1.
problem.setStateInfo('/slider/position/value', osim.MocoBounds(-5, 5),
                     osim.MocoInitialBounds(0), osim.MocoFinalBounds(1))
# Initial and final speed must be 0. Use compact syntax.
problem.setStateInfo('/slider/position/speed', [-5, 5], [0], [0])

# Applied force must be between -50 and 50.
problem.setControlInfo('/actuator', osim.MocoBounds(-50, 50))

# Cost.
# -----
problem.addGoal(osim.MocoFinalTimeGoal())

# Configure the solver.
# =====================
solver = study.initTropterSolver()
solver.set_num_mesh_intervals(100)

# Now that we've finished setting up the study, print it to a file.
if not os.path.exists("Results") or not os.path.isdir("Results"): # Create the folder if it doesn't exist
    print("Create folder Results")
    os.makedirs("Results", exist_ok=True)
study.printToXML('Results/sliding_mass.omoco')

# Solve the problem.
# ==================
solution = study.solve()

solution.write('Results/sliding_mass_solution.sto')

# Plot the torque
table_solution = osim.TimeSeriesTable(
    "Results/sliding_mass_solution.sto")
actuator = table_solution.getDependentColumn("/actuator").to_numpy()
position = table_solution.getDependentColumn("/slider/position/value").to_numpy()
speed = table_solution.getDependentColumn("/slider/position/speed").to_numpy()
time = table_solution.getIndependentColumn()

plot = False
if plot:
    if os.getenv('OPENSIM_USE_VISUALIZER') != '0':
        try:
            import pylab as pl

            plot = True
        except ImportError:
            pl = None
            print('Skipping plotting')

    pl.plot(time, position,
            label='position', lw=4)

    pl.plot(time, speed,
            label='speed', lw=4)

    pl.legend()
    pl.show()

if os.getenv('OPENSIM_USE_VISUALIZER') != '0':
    study.visualize(solution)
