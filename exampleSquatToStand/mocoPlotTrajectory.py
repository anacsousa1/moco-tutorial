import opensim as osim
import matplotlib.pyplot as plt
import numpy as np
from opensim import MocoTrajectory


# Plot a MocoTrajectory. Optionally, specify a second trajectory and names
# for the trajectories.
def mocoPlotTrajectory(*args):
    args_list = list(args)

    if isinstance(args_list[0], str):
        traj_a: MocoTrajectory = osim.MocoTrajectory(args_list[0])
    
    if len(args) > 1 and isinstance(args_list[1], str):
        traj_b = osim.MocoTrajectory(args_list[1])

    label_a = args_list[2] if len(args) == 4 else 'A'
    label_b = args_list[3] if len(args) == 4 else 'B'
    
    # # States.
    state_names = list(traj_a.getStateNames())
    num_states = len(state_names)
    dim = np.sqrt(num_states)
    if dim == np.ceil(dim):
        num_rows = int(dim)
        num_cols = int(dim)
    else:
        num_cols = int(np.min([num_states, 4]))
        num_rows = int(np.floor(num_states / 4))
        if not num_states % 4 == 0:
            num_rows += 1
    
    fig = plt.figure()
    for i in np.arange(num_states):
        ax = fig.add_subplot(num_rows, num_cols, int(i+1))
        ax.plot(traj_a.getTimeMat(),
                traj_a.getStateMat(state_names[i]), color='red',
                linewidth=3, label=label_a)

        if len(args) > 1:
            ax.plot(traj_b.getTimeMat(),
                    traj_b.getStateMat(state_names[i]),
                    color='blue', linestyle='--',
                    linewidth=2.5, label=label_b)
        
        state_name = state_names[i]
        ax.set_title(state_name[11:])
        ax.set_xlabel('time (s)')
        ax.set_xlim(0, 1)
        if 'value' in state_name:
            ax.set_ylabel('position (rad)')
        elif 'speed' in state_name:
            ax.set_ylabel('speed (rad/s)')
        elif 'activation' in state_name:
            ax.set_ylabel('activation (-)')
            ax.set_ylim(0, 1)
        
        if i == 0 and len(args) > 1:
            ax.legend(loc='best')

    plt.show()

    # # Controls.
    control_names = list(traj_a.getControlNames())
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

    fig = plt.figure()
    for i in np.arange(num_controls):
        ax = fig.add_subplot(num_rows, num_cols, int(i+1))
        y_a = traj_a.getControlMat(control_names[i])
        ax.plot(traj_a.getTimeMat(), y_a, color='red',
                linewidth=3, label=label_a)
        if len(args) > 1:
            y_b = traj_b.getControlMat(control_names[i])
            ax.plot(traj_b.getTimeMat(), y_b, color='blue',
                    linestyle='--', linewidth=2.5,
                    label=label_b)
       
        ax.set_title(control_names[i])
        ax.set_xlabel('time (s)')
        ax.set_ylabel('value')
        ax.set_xlim(0, 1)
        if np.max(y_a) <= 1 and np.min(y_a) >= 0:
            fix_y_lim = True
            if len(args) > 1 and (np.max(y_b) > 1 or
                                  np.min(y_b) < 0):
                fix_y_lim = False
            
            if fix_y_lim:
                ax.set_ylim(0, 1)

        if i == 0 and len(args) > 1:
            ax.legend(loc='best')

    fig.tight_layout()
    plt.show()
    plt.close()
