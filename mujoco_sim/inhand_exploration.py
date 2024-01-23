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

import scipy.io as sio

import include.allegro_hand_sym as allegro_hand_ik

hand_ik = allegro_hand_ik.Robot(right_hand=False, use_fingers=[1, 1, 1, 1],
                                tip_link=['link_15_tip', 'link_3_tip', 'link_7_tip', 'link_11_tip'])

# obj_name = 'apc_1'
# obj_name = 'apc_2'
# obj_name = 'apc_red_bowl'
# obj_name = 'bunny'              #  for this mesh, a 90 degree rotation along x axis, already set in .xml
# obj_name = 'apc_redcup'
# obj_name = 'finger_tip'
obj_name = 'finger_tip2'
# obj_name = 'apc_2x2'

if obj_name in ['bunny', 'apc_red_bowl', 'apc_redcup']:
    scalingFactor = 2
elif obj_name == 'apc_1':
    scalingFactor = 2.5
elif obj_name == 'apc_2':
    scalingFactor = 1
else:
    scalingFactor = 1
    # raise NotImplementedError

# thumb, index, middle, ring
# red,  green, blue, yellow
contact_color = [np.array([1, 0, 0, 1]), np.array([0, 1, 0, 1]), np.array([0, 0, 1, 1]), np.array([1, 1, 0, 1])]
contact_points = [[], [], [], []]
finger_tip_color = [np.array([0.5, 0, 0, 1]), np.array([0, 0.5, 0, 1]), np.array([0, 0, 0.5, 1]), np.array([0.5, 0.5, 0, 1])]

xml_path = 'description/inhand_exploration_' + obj_name + '.xml'

model = load_model_from_path(xml_path)

sim = MjSim(model)  # MjSim represents a running simulation including its state.
viewer_ = True
if viewer_:
    # if report an error, `_hide_overlay=False` can be deleted.
    viewer = MjViewer(sim, _hide_overlay=True)
else:
    viewer = None
    # No viewer
    offscreen = mujoco_py.MjRenderContext(sim=sim, device_id=0, offscreen=True, opengl_backend='glfw')

# use 6 joint to stimulate the motion of palm
if obj_name == 'apc_2x2':
    pieces_name = 'apc_2'
else:
    pieces_name = obj_name
r = controllers_utils.Robot(sim, 0, viewer=viewer, data_log=False, print_contact=False, exploration_list=[pieces_name], frame_view=['allegro_base'])

data = np.zeros([32])
tips_normal_data = np.zeros([4, 3])
current_contact_pos = np.zeros([4, 3])
current_contact_label = np.zeros(4)

# for one step, store all contacts during the interpolation trajectory of hand
contacts_one_step = [[], [], [], []]


def pose_joints_cb(state):
    global data, contacts_one_step
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", state.data)
    # print(r.qh)
    data = np.array(state.data)
    contacts_one_step = [[], [], [], []]
    # print(data)


# def tips_normal(state):
#     global tips_normal_data
#     tips_normal_data = np.array(state.data).reshape(4, 3)  # out of the surface
#     # print(tips_normal_data.reshape(4, 3))

# There are three sources of normal direction, which should be used for projection of moving direction.
# 1. the last contact position and last fingertip position
# 2. the last contact force direction
# 3. the normal from GP


# data order   [T[:,0],T[:,1],T[:,2],T[:,3], thumb index middle ring


rospy.init_node('mujoco', anonymous=True)
rospy.Subscriber("pose_joints", Float64MultiArray, pose_joints_cb)
# rospy.Subscriber("tips_normal", Float64MultiArray, tips_normal)

t0 = time.time()

initial_rotation = rot.quat_mul(rot.euler2quat([0, np.pi / 2, 0]), rot.euler2quat([-np.pi / 2, 0, 0]))

# displacement of hand pose, the palm frame with respect to the wrist frame
p_palm2wrist = np.array([0.012, 0, 0.1075, 1, 0, 0, 0])
p_obj = r.get_body_pose(obj_name)  # the initial pose of the object

# because in Matlab, the obj bottom is at the xoy plane of world frame
if obj_name == 'finger_tip2':
    path = '../database/experimental_objects/' + 'finger_tip' + '.mat'
else:
    path = '../database/experimental_objects/' + obj_name + '.mat'
mat = sio.loadmat(path)
point_cloud = mat['pointCloud'] * scalingFactor
p_center = np.mean(point_cloud, axis=0)  # get the center of all point cloud
p_obj_transfer = np.concatenate([-p_center, np.array([1, 0, 0, 0])])
p_obj_transfer2 = rot.pose_mul(p_obj,
                               p_obj_transfer)  # applied by this transfer, the matlab world frame is aligned with the mujoco world frame
# so, if Matlab sends the frame of the palm, use p_obj_transfer2 @ p_palm to get the palm pose in Mujoco.
p_wrist2palm = rot.pose_inv(p_palm2wrist)

# check contact points:
# (1) Matlab
# (2) Mujoco
contacts_mat = sio.loadmat('../src/sampledCloud_matlab_sim.mat')['sampledCloud'] / 1000
contacts_muj = sio.loadmat('../src/sampledCloud_mujoco_sim.mat')['sampledCloud'] / 1000


## this loop is to make sure that the transformation is correct.

# (in the viewer, the point cloud should be exactly at the surface of mujoco stl model).
# while 1:
#     r.run()
#     # r.view_obj_mat(point_cloud,  size=0.001)
#     r.view_obj_mat(contacts_mat, size=0.005, nums=contacts_mat.shape[0], color=np.array([0, 0, 1, 0.5]))
#     r.view_obj_mat(contacts_muj, size=0.005, nums=contacts_muj.shape[0], color=np.array([0, 1, 0, 0.5]))

def get_desired_tip_position(x: np.ndarray, qh: np.ndarray) -> list:
    poses, _ = hand_ik.forward_kine(qh)
    tip_positions = [rot.pose_mul(x, pose)[:3] for pose in poses]
    return tip_positions


q0 = np.zeros(16)
q0[[1, 5, 9, 13]] = np.pi / 2  # initial joints, to check if the frames are aligned.
T0 = np.array([0, 0, 0.366, 1, 0, 0, 0])  # palm wrt world
T0[3:] = initial_rotation
T0_wrist = rot.pose_mul(T0, p_wrist2palm)

while np.linalg.norm(data) < 0.001:
    time.sleep(0.2)  # wait to receive data.

tip_site = [r.site[i] for i in [1, 3, 6, 9]]
tip_site1 = [r.site[i] for i in [2, 4, 7, 10]]
kp_iiwa = 100
kp_hand = 0.4 * 0.1
kd_iiwa = 2 * np.sqrt(kp_iiwa)
kd_hand = 2 * np.sqrt(kp_hand) * 0.01

kp = np.identity(23) * kp_hand
kp[range(0, 7), range(0, 7)] = kp_iiwa
kd = np.identity(23) * kd_hand
kd[range(0, 7), range(0, 7)] = kd_iiwa

q_rest = np.zeros(23)  # rest joints, to be set in null space
q_rest[7] = 0.5  # thumb
q_rest[[8, 9, 10]] = 0.4
q_rest[[12, 13, 14, 16, 17, 18, 20, 21, 22]] = 0.3
first_pose = True

mocap = True
feedback = True
while 1:

    x1 = rot.Mat2pose(data[:16])  # rotation matrix . This is the ori of palm (base_link_left)!!

    q1 = rot.quat_mul(x1[3:], initial_rotation)  # add rotation at the orientation from Matlab, to align frame of hand.

    p_palm2world = np.concatenate([x1[:3], q1])
    # p_palm2world = rot.pose_mul(p_obj_transfer2, p_matlab)
    p_wrist2world = rot.pose_mul(p_palm2world, p_wrist2palm)  # the pose of allegro base in world frame.

    if first_pose:
        if mocap:
            r.sim.data.mocap_pos[:] = p_wrist2world[:3]  # apply the pose to the base link of allegro amount
            r.sim.data.mocap_quat[:] = p_wrist2world[3:]
        first_pose = False  # comment this line to have the wrist moving discontinuously.
    else:

        # use DS to generate traj
        if mocap:
            x = r.sim.data.mocap_pos[:].flatten()
            q = r.sim.data.mocap_quat[:].flatten()
        else:
            x = r.x[:3]
            q = r.x[3:]
        p_wrist = np.concatenate([x, q])
        p_palm = rot.pose_mul(p_wrist, p_palm2wrist)
        x = p_palm[:3]
        q = p_palm[3:]

        xd = p_palm2world[:3]
        qd = p_palm2world[3:]

        # print(x)
        # print(q)
        if mocap:
            vel_scale = 0.2 * 1.5
        else:
            vel_scale = 30
        a = 0.3 * vel_scale
        b = 10 * vel_scale
        dt = 0.002
        a = a / (np.linalg.norm(x - xd) + 0.002)
        dx = -1 * a * (x - xd)
        if q[0] < 0:
            q = -q
        if qd[0] < 0:
            qd = -qd
        b = b / (rot.ori_dis(q, qd) + 1 / 180. * np.pi)
        w = -1 * b * rot.log(rot.quat_mul(q, rot.quat_conjugate(qd)))

        # integral
        x_new = x + dx * dt
        dq = rot.axisangle2quat(w * dt)
        q_new = rot.quat_mul(dq, q)

        # `mocap` means that we ignore dynamics of the body and directly set the pose of it.
        p_palm2world = np.concatenate([x_new, q_new])
        p_wrist2world = rot.pose_mul(p_palm2world, p_wrist2palm)  # the pose of allegro base in world frame.

        if mocap:
            r.sim.data.mocap_pos[:] = p_wrist2world[:3]  # apply the pose to the base link of allegro
            r.sim.data.mocap_quat[:] = p_wrist2world[3:]

    qh = data[16:32]  # please notice the order of fingers, the same as allegro_left_bodies: [thumb index middle ring]
    qh, violate1 = controllers_utils.hand_joint_limit(qh)
    qa, violate2 = controllers_utils.hand_joint_limit(r.qh)
    if violate2:
        r.set_left_hand_joints(qa)

    # print(qh)
    # print('real', r.qh)
    # print('q error',          qh - r.qh)

    # if [] in r.exploration_contact: # if no contact for any fingertip, go down under get contact
    #     r.sim.data.mocap_pos[:] = r.sim.data.mocap_pos[:].flatten()  - np.array([0, 0, 0.0001])

    u_add = np.zeros(16)
    # F_threshold = 1
    # for i, pos_F in enumerate(r.exploration_contact):
    #     if pos_F != []:
    #         F = pos_F[3:]
    #         delta_F = F - F/np.linalg.norm(F) * F_threshold
    #         tau = r.J_site[i][:3, :].T @ delta_F
    #         u_add[i*4:(i+1)*4] = tau

    # r.left_hand_move(qh=qh, scale=1, u_add=u_add, iiwa=True, pose=p_wrist2world)  # the fingers are impedance-controlled
    # print(r.x[:3] - p_wrist2world[:3])
    # get normal direction
    r.contact_view()
    iiwa_acc = r.iiwa_Cartesion_impedance(p_wrist2world)
    iiwa_acc = np.concatenate([iiwa_acc, np.zeros(16)])
    # for hand, only track to the desired finger tips
    F = np.zeros(23)
    if mocap:
        wrist_pose = p_wrist2world
    else:
        wrist_pose = r.x
    tip_pos = get_desired_tip_position(wrist_pose, qh)
    tip_pos0 = tip_pos.copy()
    threshold_one_step = 0.005
    for i, tip in enumerate(tip_site):
        pos = r.exploration_contact[i]
        if len(pos) != 0:
            current_contact_pos[i, :] = pos[:3]  # store the current contact position
            current_contact_label[i] = 0
            if len(contacts_one_step[i]) == 0:
                contacts_one_step[i] = pos[:3].reshape(1, -1)
            else:
                if np.linalg.norm(contacts_one_step[i][-1, :] - pos[0:3]) > threshold_one_step:
                    contacts_one_step[i] = np.vstack([contacts_one_step[i], pos[0:3].reshape(1, -1)])

            delta_pos = np.copy(r.xh[i, :3]) - current_contact_pos[i, :]
            tips_normal_data[i, :] = delta_pos / (np.linalg.norm(delta_pos) + 1e-10)
            # project tip_pos to the plane of the tip_normal
            p = tip_pos[i] - r.xh[i, :3]
            tip_pos[i] = r.xh[i, :3] + p - np.sum(tips_normal_data[i, :] * p) * tips_normal_data[i, :] * 0


        else:
            print(i, 'has no contact.')
            # current_contact_pos[i, :] = r.xh[i, :3]
            current_contact_label[i] = 1
        r.view_vector(current_contact_pos[i, :], 0.1 * tips_normal_data[i, :], rgba=np.array([0.4, 0.8, 1, 0.7]))
        # if np.linalg.norm(current_contact_pos[i, :]) <= 1e-5:
        #     pass
        # else:
        if current_contact_label[i] == 0:  # in contact
            # Fd = 0.2 - pos[3:6]
            if np.linalg.norm(pos[3:6]) > 1:
                Fd = -0.5
            else:
                Fd = 0.5
            kp_tip = 10
            kp_gain = 1
            kd_gain = 1
        else:  # without contact
            Fd = 1
            kp_tip = 1
            kp_gain = 0.2
            kd_gain = 0.2
        J = r.J_site[tip]
        pos_error = tip_pos[i] - r.xh[i, :3]
        F += J.T @ (kp_tip * pos_error)  # fingertip position tracking
        F += J.T @ (- Fd * tips_normal_data[i, :])  # additional force
        # with null space
        N = np.eye(23) - J.T.dot(np.linalg.solve(J.dot(J.T) + 1e-10 * np.eye(3), J))
        q_rest[:7] = r.q  # ignore the null space of iiwa, otherwise iiwa will try to go zero joints
        acc_null_rest = kp / 2 * kp_gain @ (q_rest - r.q_) + kd / 10 * kd_gain @ (0 - r.dq_)  # close to q_rest
        F_null = N @ acc_null_rest
        F += F_null
    # for the Knuckle
    F_knu = np.zeros(23)
    # for i, tip in enumerate(tip_site1):
    #     J = r.J_site[tip]

    tau = iiwa_acc + r.C_ + F + F_knu
    r.send_torque(tau)

    # visualize tip_pos and tip_pos0
    for i in range(4):
        r.view_obj_mat(tip_pos[i], size=0.004, color=finger_tip_color[i], nums=0)
        r.view_obj_mat(tip_pos0[i], size=0.002, color=finger_tip_color[i], nums=0)


    contacts_one_step_ = []
    for i, contact in enumerate(contacts_one_step):
        if len(contacts_one_step[i]) != 0:
            contacts_one_step_.append(contact)

    # send joints to Matlab
    if feedback:
        pub = rospy.Publisher('contacts', Float64MultiArray, queue_size=20)
        states = Float64MultiArray()  # q0, x0 R0 in solve_3D_sym2matlab.m
        p_palm2world_mat = np.copy(p_palm2world)
        p_palm2world_mat[3:] = rot.quat_mul(p_palm2world_mat[3:], rot.quat_conjugate(initial_rotation))
        if len(contacts_one_step_) != 0:
            states.data = np.concatenate(
                [p_palm2world_mat, r.qh, current_contact_pos.flatten(), tips_normal_data.flatten(),
                 current_contact_label, np.vstack(contacts_one_step_).flatten()])
            # print(np.vstack(contacts_one_step_))
        else:
            states.data = np.concatenate(
                [p_palm2world_mat, r.qh, current_contact_pos.flatten(), tips_normal_data.flatten(),
                 current_contact_label])
        pub.publish(states)
        # print(rot.pose2T(p_palm2world))
        # print(current_contact_pos)
        # print(tips_normal_data)
        # print(p_wrist2world[:3])
    # print(x1)
    # r.moveto_attractor(p_wrist2world, qh, couple_hand=False)
    # time.sleep(0.002)
    # print(r.hand_contact_force)
    # print(r.exploration_contact)
    threshold = 0.02
    for i, pos in enumerate(r.exploration_contact):
        if len(pos) != 0:
            # assert pos[1] == r.finger_tip[i]

            if len(contact_points[i]) != 0:
                if np.linalg.norm(contact_points[i][-1, :] - pos[0:3]) > threshold:
                    contact_points[i] = np.vstack([contact_points[i], pos[0:3].reshape(1, -1)])
            else:
                contact_points[i] = pos[0:3].reshape(1, -1)

        if len(contact_points[i]) != 0:
            r.view_obj_mat(contact_points[i], size=0.005, color=contact_color[i], nums=0)

            # if contact_points[i]:
            #     pass
            #     # if np.linalg.norm(contact_points[i][-1] - pos[0]) > 0.05:
            #     #     contact_points[i].append(pos[0])
            # else:
            #     contact_points[i].append(pos[0])

    # for i, pos in enumerate(contact_points):
    #     if pos:
    #         r.view_obj_mat(np.vstack(pos), color=contact_color[i], size=0.01, interval=1)
    # print([len(c) for c in contact_points])

    # print(r.exploration_contact)