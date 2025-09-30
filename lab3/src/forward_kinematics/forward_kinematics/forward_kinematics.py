#!/usr/bin/env python3

import numpy as np
try:
    from . import kin_func_skeleton as kfs
except ImportError:
    import kin_func_skeleton as kfs 

def ur7e_foward_kinematics_from_angles(joint_angles):
    # Points on each joint axis in the zero config
    q0 = np.zeros((3, 6))
    q0[:, 0] = [0., 0., 0.1625]        # shoulder pan joint - shoulder_link
    q0[:, 1] = [0., 0., 0.1625]        # shoulder lift joint - upper_arm_link
    q0[:, 2] = [0.425, 0., 0.1625]     # elbow_joint - forearm_link
    q0[:, 3] = [0.817, 0.1333, 0.1625] # wrist 1 - wrist_1_link
    q0[:, 4] = [0.817, 0.1333, 0.06285] # wrist 2 - wrist_2_link
    q0[:, 5] = [0.817, 0.233, 0.06285] # wrist 3 - wrist_3_link

    # Axis vector of each joint axis in the zero config
    w0 = np.zeros((3, 6))
    w0[:, 0] = [0., 0., 1]   # shoulder pan joint
    w0[:, 1] = [0, 1., 0]    # shoulder lift joint
    w0[:, 2] = [0., 1., 0]   # elbow_joint
    w0[:, 3] = [0., 1., 0]   # wrist 1
    w0[:, 4] = [0., 0., -1]  # wrist 2 
    w0[:, 5] = [0., 1., 0]   # wrist 3

    # Calculate twists for each joint
    xi = np.zeros((6, 6))  # 6x6 matrix where each column is a twist
    
    for i in range(6):
        v_i = -np.cross(w0[:, i], q0[:, i])
        xi[:, i] = np.concatenate([v_i, w0[:, i]])
    
    # Calculate product of exponentials
    g = kfs.prod_exp(xi, joint_angles)
    
    # Transformation from base_link to wrist_3_link in zero config
    # This represents the final transformation to the end-effector frame
    g_st0 = np.array([[-1., 0., 0., 0.817],
                      [0., 0., 1., 0.233], 
                      [0., 1., 0., 0.06285],
                      [0., 0., 0., 1.]])
    
    # Final transformation matrix
    T = g @ g_st0
    
    return T    

def ur7e_forward_kinematics_from_joint_state(joint_state):
    order = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
    ]
    angles = np.zeros(6)
    for i, name in enumerate(order):
        idx = joint_state.name.index(name)
        angles[i] = joint_state.position[idx]
    return ur7e_foward_kinematics_from_angles(angles)