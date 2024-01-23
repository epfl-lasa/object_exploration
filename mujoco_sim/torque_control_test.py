import time
import rospy
from std_msgs.msg import Float64MultiArray

from mujoco_py import load_model_from_xml, MjSim, MjViewer, load_model_from_path
import mujoco_py
import numpy as np
# import matplotlib.pyplot as plt
# import os
# import sys
import include.controllers_utils as controllers_utils
import include.rotations as rot
import include.allegro_hand_sym as allegro_hand_ik

# obj_name = 'apc_1'
# obj_name = 'apc_2'
obj_name = 'finger_tip'
# obj_name = 'apc_red_bowl'
# obj_name = 'bunny'
# obj_name = 'apc_redcup'

xml_path = 'description/inhand_exploration_' + obj_name + '.xml'
hand_ik = allegro_hand_ik.Robot(right_hand=False, use_fingers=[1, 1, 1, 1],
                                tip_link=['link_15_tip', 'link_3_tip', 'link_7_tip', 'link_11_tip'])

model = load_model_from_path(xml_path)

sim = MjSim(model)  # MjSim represents a running simulation including its state.
viewer_ = True
if viewer_:
    # if report an error, `_hide_overlay=False` can be deleted.
    viewer = MjViewer(sim, _hide_overlay=False)
else:
    viewer = None
    # No viewer
    offscreen = mujoco_py.MjRenderContext(sim=sim, device_id=0, offscreen=True, opengl_backend='glfw')

r = controllers_utils.Robot(sim, 0, viewer=viewer, data_log=False, print_contact=False, exploration_list=[obj_name])
xd = r.x

tmp = np.array([0, 0, 0, 0])
q_middle = tmp + np.array([0, 0.6, 0.3, 0.3])
q_ring = tmp + np.array([0, 0.3, 0.3, 0.3])
q_index = tmp + np.array([0, 0.2, 0.3, 0.3])
q_thumb = np.array([0, 0.2, 0.4, 0.])
qh = np.concatenate([q_thumb, q_index, q_middle, q_ring])


# poses, jacs = hand_ik.forward_kine(qh)


def get_desired_tip_position(x: np.ndarray, qh: np.ndarray) -> list:
    poses, _ = hand_ik.forward_kine(qh)
    # tip_positions = []
    # for pose in poses:
    #     tip_pose = rot.pose_mul(x, pose)
    #     tip_positions.append(tip_pose[:3])
    tip_positions = [rot.pose_mul(x, pose)[:3] for pose in poses]
    return tip_positions


q0 = np.concatenate([r.q, qh])

q_rest = np.zeros(23)  # rest joints, to be set in null space
q_rest[7] = 0.5  # thumb
# 1: joint space, PD + computed torque control
# 2: joint space impedance control
# 3: joint space impedance control with additional contact force
# 4: Cartesian space impedance control for iiwa
controller = 4
if controller == 1:
    kp_iiwa = 100
    kd_iiwa = 2 * np.sqrt(kp_iiwa)
    kp_hand = 0.4 * 10 * 4
    kd_hand = 2 * np.sqrt(kp_hand) * 0.01


elif controller in [2, 3, 4]:
    kp_iiwa = 100
    kp_hand = 0.4 * 0.1
    kd_iiwa = 2 * np.sqrt(kp_iiwa)
    kd_hand = 2 * np.sqrt(kp_hand) * 0.01

kp = np.identity(23) * kp_hand
kp[range(0, 7), range(0, 7)] = kp_iiwa
kd = np.identity(23) * kd_hand
kd[range(0, 7), range(0, 7)] = kd_iiwa

if controller == 4:
    kp1 = np.array([300, 200.])
    kd1 = np.sqrt(kp1) * 2 * 2. * 3

tip_site = [r.site[i] for i in [1, 3, 6, 9]]
tip_site1 = [r.site[i] for i in [2, 4, 7, 10]]
while True:
    # r.iiwa_hand_go(xd, qh)
    # r.left_hand_move(qh, scale=1)
    # print(np.max(np.abs(r.dqh)))
    # error = r.qh - qh
    # qh_error = np.linalg.norm(r.qh - qh) * 180 / np.pi
    # print(qh_error)
    # r.run()

    # (1) joint space, PD + computed torque control
    if controller == 1:
        acc = 0 + kp @ (q0 - r.q_) + kd @ (0 - r.dq_)
        tau = r.M_ @ acc + r.C_
    elif controller == 2:
        acc = kp @ (q0 - r.q_) + kd @ (0 - r.dq_)
        tau = acc + r.C_
    elif controller == 3:
        acc = kp @ (q0 - r.q_) + kd @ (0 - r.dq_)
        tau = acc + r.C_ + r.J_site[6].T @ np.array([0, 0, -0.05]) + \
              r.J_site[9].T @ np.array([0, 0, -0.5])
        # r.J_site[10].T @ np.array([0, 0, -0.5])
    elif controller == 4:
        iiwa_acc = r.iiwa_Cartesion_impedance(xd, kp=kp1)
        iiwa_acc = np.concatenate([iiwa_acc, np.zeros(16)])
        # for hand, only track to the desired finger tips
        F = np.zeros(23)
        tip_pos = get_desired_tip_position(r.x, qh)
        for i, tip in enumerate(tip_site):
            J = r.J_site[tip]
            pos_error = tip_pos[i] - r.xh[i, :3]
            F += J.T @ (10 * pos_error)  # fingertip position tracking
            # with null space
            N = np.eye(23) - J.T.dot(np.linalg.solve(J.dot(J.T) + 1e-10 * np.eye(3), J))
            q_rest[:7] = r.q  # ignore the null space of iiwa, otherwise iiwa will try to go zero joints
            acc_null_rest = kp/4 @ (q_rest - r.q_) + kd/10 @ (0 - r.dq_)  # close to q_rest
            F_null = N @ acc_null_rest
            F += F_null
        # for the Knuckle
        F_knu = np.zeros(23)
        for i, tip in enumerate(tip_site1):
            J = r.J_site[tip]



        tau = iiwa_acc + r.C_ + F + F_knu

    # tip_pos = get_desired_tip_position(r.x, r.qh)
    # for i in range(4):
    # it shoule be equal to zero
    #     print(r.xh[i,:3] - tip_pos[i]) # this is to validate if the fkine is correct.

    r.send_torque(tau)
