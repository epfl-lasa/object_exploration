import numpy as np
import mujoco_py
from mujoco_py.generated import const

from enum import Enum, auto
import quaternion
# https://github.com/moble/quaternion
# https://quaternion.readthedocs.io/en/latest/

import time
from . import rotations as rot
import scipy.io as sio


# # from trac_ik_python.trac_ik_wrap import TRAC_IK
#
# from trac_ik_python.trac_ik import IK


class TrajectoryProfile(Enum):
    SPLINE3 = auto()
    SPLINE5 = auto()
    STEP = auto()


class CtrlType(Enum):
    # Controllers
    INDEP_JOINT_PID = auto()  # PID + Gravity for each joint
    INV_DYNAMICS_JOINT_PD = auto()  # computed torque + PD in joint space
    INV_DYNAMICS_OP_SPACE = auto()  # inverse dynamics in operational space
    IMPEDANCE_OP_SPACE = auto()  # impedance control in operational space
    INV_KINEMASTICS = auto()


def hand_joint_limit(q):
    # thumb index middle ring
    qh = np.copy(q)
    lb = [0.3635738998060688, -0.20504289759570773, -0.28972295140796106, -0.26220637207693537]
    ub = [1.4968131524486665, 1.2630997544532125, 1.7440185506322363, 1.8199110516903878]

    violate = False
    for i in range(0, 4):
        if qh[i] < lb[i % 4]:
            qh[i] = lb[i % 4]
            violate = True
        if qh[i] > ub[i % 4]:
            qh[i] = ub[i % 4]
            violate = True
    lb = [-0.59471316618668479, -0.29691276729768068, -0.27401187224153672, -0.32753605719833834]
    ub = [0.57181227113054078, 1.7367399715833842, 1.8098808147084331, 1.71854352396125431]
    for i in range(4, 16):
        if qh[i] < lb[i % 4]:
            qh[i] = lb[i % 4]
            violate = True
        if qh[i] > ub[i % 4]:
            qh[i] = ub[i % 4]
            violate = True

    return qh, violate


def torque_limit(torque):
    iiwa_torque_limit = np.array([320, 320, 176, 176, 110, 40, 40]) / 2.  # iiwa 14
    allegro_torque_limit = 0.13  # 0.7 * 1.0

    for i in range(7):
        if np.abs(torque[i]) > iiwa_torque_limit[i]:
            # print('control torque:', torque[:self.start_hand_joint])
            torque[i] = torque[i] / np.abs(torque[i]) * iiwa_torque_limit[i]

    if len(torque) == 23:
        for i in range(7, 23):
            if np.abs(torque[i]) > allegro_torque_limit:
                # print('control torque:', torque[self.start_hand_joint:])
                torque[i] = torque[i] / np.abs(torque[i]) * allegro_torque_limit
    elif len(torque) == 16:
        for i in range(16):
            if np.abs(torque[i]) > allegro_torque_limit:
                torque[i] = torque[i] / np.abs(torque[i]) * allegro_torque_limit

    return torque


class Robot:
    def __init__(self, sim_handle: mujoco_py.MjSim, controller=0, exploration_list=None, viewer=None, data_log=False,
                 print_contact=False, show_contact_F=True, start_hand_joint=7, frame_view=None):
        self.sim = sim_handle
        self.n = sim_handle.model.nu
        self.nv = sim_handle.model.nv
        self.dt = sim_handle.model.opt.timestep
        self.name_tcp = self.sim.model.site_id2name(0)
        self.iiwa_joint_names = ['iiwa_joint_' + str(i+1) for i in range(7)]
        self.hand_joint_names = ['thumb_joint_' + str(i) for i in range(4)] + \
                                ['index_joint_' + str(i) for i in range(4)] + \
                                ['middle_joint_' + str(i) for i in range(4)] + \
                                ['ring_joint_' + str(i) for i in range(4)]
        self.iiwa_joint_id = [self.sim.model.joint_name2id(i) for i in self.iiwa_joint_names]
        self.hand_joint_id = [self.sim.model.joint_name2id(i) for i in self.hand_joint_names]
        self.iiwa_hand_joint_id = self.iiwa_joint_id + self.hand_joint_id

        if start_hand_joint == 7:
            link_1 = self.sim.model.body_name2id('iiwa_link_1')
            self.name_bodies = [self.sim.model.body_id2name(i) for i in range(link_1, link_1 + 7)]
            self.mass_links = self.sim.model.body_mass[link_1:link_1 + 7]  # iiwa_link_0 to iiwa_link_7
        self.gravity_comp_sim = mujoco_py.MjSim(self.sim.model)
        self.viewer = viewer
        self.log = data_log
        self.q_last = self.q_
        self.control_torque = None
        self.print_contact = print_contact

        self.start_hand_joint = start_hand_joint
        self.frame_view = frame_view

        self.finger_tip = ['thumb_tip', 'index_tip', 'middle_tip', 'ring_tip']
        # the objects to be grasped
        if exploration_list is None:
            self.obj_explored = ['cylinder1', 'cylinder2']
        else:
            self.obj_explored = exploration_list
        self.exploration_obj_str_num = [len(i) for i in exploration_list]
        # print(self.exploration_obj_str_num)
        self.exploration_contact = [[], [], [],
                                    []]  # (n, 9) n is the num of contact pair. for each row  [contact position, contact force, torque] in world frame
        self.obj_num = len(self.obj_explored)

        self.site = ['ee_site', 'thumb_site', 'thumb_site1',
                     'index_site', 'index_site1', 'index_site2',
                     'middle_site', 'middle_site1', 'middle_site2',
                     'ring_site', 'ring_site1', 'ring_site2']
        # self.obj_joint = [i + ':joint' for i in self.obj_explored]


            # self.obj_joint_id = [self.sim.model.joint_name2id(i) for i in self.obj_joint]
        # print(self.obj_joint_id)

        # self.hand_contact_force_body = ['index_tip', 'middle_tip', 'ring_tip', 'thumb_tip', 'palm_link']
        self.hand_contact_force = {'index_tip': 0, 'middle_tip': 0, 'ring_tip': 0, 'thumb_tip': 0, 'palm_link': 0}
        self.hand_contact_force_norm = np.zeros([5])
        self.hand_contact_force_norm_filter = np.zeros([1, 5])
        self.hand_contact = np.zeros([5])
        self.stored_force = np.empty([0, 3])
        self.reset_contact_force()
        # contact force at index/middle/ring/index/palm.

        # reset the initial joints
        self.q0 = np.array([0, 0, 0, -np.pi / 2, np.pi / 2, 0, -np.pi / 2])
        self.q0 = np.array([-0.32032486, 0.02707055, -0.22881525, -1.42611918, 1.38608943, 0.5596685, -1.34659665])
        joint_name = 'iiwa_joint_'
        if start_hand_joint == 7:
            for i in range(7):
                tmp = joint_name + str(i + 1)
                self.sim.data.set_joint_qpos(tmp, self.q0[i])
        self.sim.data.set_joint_qpos('thumb_joint_0', 1)  # set the initial angle of the joint.
        self.sim.step()  # run one step to update all position-based state

        if self.log:
            # state value
            self.log_x = []
            self.log_q = []
            self.log_dq = []
            self.log_u = []
            # desired value
            self.log_xd = []
            self.log_qd = []
            self.log_ud = []
            self.log_dqd = []
            self.log_time = []
        self.qd = None

    def run(self):
        # print(self.M_.shape)
        # F = self.M_ @ self.ddq_
        # self.send_torque(self.C_ + self.M_ @ self.ddq_)
        self.send_torque(self.C_)

        # print(self.sim.data.qpos)

    def set_initial_hand_joint(self, qh):
        hand_joint_name = ['index_joint_', 'middle_joint_', 'ring_joint_', 'thumb_joint_']
        for i in range(7, 23):
            j = (i - 7) // 4
            k = (i - 7) % 4
            self.sim.data.set_joint_qpos(hand_joint_name[j] + str(k), qh[i - 7])

    def set_initial_joint(self, q, qh=None):
        """
        :param q:  (7,)
        :param qh: (16,)
        :return:
        """
        # qpos is a system vector with all positional degrees of freedom.
        # free joints have 7 numbers in that vector (3D position followed by 4D quaterion orientation).
        # ball joints have 4 numbers (quaternion).
        joint_name = 'iiwa_joint_'
        for i in range(7):
            tmp = joint_name + str(i + 1)
            self.sim.data.set_joint_qpos(tmp, q[i])
        if qh is not None:
            hand_joint_name = ['index_joint_', 'middle_joint_', 'ring_joint_', 'thumb_joint_']
            for i in range(7, 23):
                j = (i - 7) // 4
                k = (i - 7) % 4
                self.sim.data.set_joint_qpos(hand_joint_name[j] + str(k), qh[i - 7])
        self.sim.step()  # run one step to update all position-based state

    def set_left_hand_joints(self, qh):
        hand_joint_name = ['thumb_joint_', 'index_joint_', 'middle_joint_', 'ring_joint_']
        for i in range(7, 23):
            j = (i - 7) // 4
            k = (i - 7) % 4
            self.sim.data.set_joint_qpos(hand_joint_name[j] + str(k), qh[i - 7])
        self.sim.step()  # run one step to update all position-based state

    def set_object_pose(self, joint, pose):
        """
        set the object pose with freejoint, you should set it only once or in a period,
         so that the object can get back its dynamics
        :param joint:    freejoint name, str
        :param pose:     (7,)  [position, quaternion]
        :return:
        """
        if len(pose) == 6:
            q0 = rot.euler2quat(pose[3:])
            pose = np.concatenate(pose[:3, q0])
        joint_id = self.sim.model.joint_name2id(joint)
        # print(joint_id)
        self.sim.data.qpos[joint_id:joint_id + 7] = pose

    def get_body_pose(self, body_name):
        x = self.sim.data.get_body_xpos(body_name)
        q = self.sim.data.get_body_xquat(body_name)
        return np.concatenate([x, q])

    def view_obj_mat(self, point_cloud, p=None, color=np.array([0, 0, 1, 0.5]), size=0.001, nums=300):
        """
        load the obj.mat by path and visualize point_cloud in Mujoco
        """
        if point_cloud.ndim == 1:
            self.viewer.add_marker(pos=point_cloud,  # position of the arrow
                                   size=np.ones([3]) * size,  # size o
                                   mat=np.identity(3),  # orientation as a matrix
                                   rgba=color,  # color of the arrow
                                   type=const.GEOM_SPHERE,  #
                                   label=''
                                   )
        # mat = sio.loadmat(path)
        # point_cloud = mat['pointCloud']
        # maxgeom: 1000
        else:
            if nums:
                a = np.linspace(0, point_cloud.shape[0], nums, endpoint=False)
                a = a.astype(int).tolist()
            else:
                a = range(0, point_cloud.shape[0])
            for i in a:
                x = point_cloud[i, :]
                if p is not None:
                    x = rot.pose2T(p) @ np.concatenate([x, [1]])
                self.viewer.add_marker(pos=x[:3],  # position of the arrow
                                       size=np.ones([3]) * size,  # size o
                                       mat=np.identity(3),  # orientation as a matrix
                                       rgba=color,  # color of the arrow
                                       type=const.GEOM_SPHERE,  #
                                       label=''
                                       )
        # self.viewer.render()

    def qua2Mat(self, quat):
        """
        Transfer quaternion [x,y,z,w] to 3x3 rotation matrix
        """
        res = np.zeros(9)
        mujoco_py.functions.mju_quat2Mat(res, quat)
        return res.reshape(3, 3)

    def send_torque(self, torque):
        """
        control joints by torque and send to mujoco, then run a step.
        input the joint control torque
        Here I imply that the joints with id from 0 to n are the actuators.
        so, please put robot xml before the object xml.
        todo, use joint_name2id to avoid this problem
        :param torque:  (n, ) numpy array
        :return:
        """

        self.q_last = self.qh
        torque = torque_limit(torque)
        self.sim.data.ctrl[:len(torque)] = torque
        self.control_torque = torque
        self.sim.step()
        if self.viewer is not None:

            self.viewer.render()
            if self.frame_view is not None:
                for i in self.frame_view:
                    self.view_frame3(i)

    def gravity_compen(self):
        # gravity compensation for 7 joints
        # Jp_shape = (3, 7)
        # comp = np.zeros((7,))
        # for body, mass in zip(self.name_bodies, self.mass_links):
        #     Jp = self.sim.data.get_body_jacp(body).reshape(Jp_shape)
        #     comp = comp - np.dot(Jp.T, self.sim.model.opt.gravity * mass)

        self.gravity_comp_sim.data.qpos[0:self.n] = self.sim.data.qpos[:self.n].copy()
        self.gravity_comp_sim.data.qvel[:self.n] = np.zeros_like(self.n)
        self.gravity_comp_sim.data.qacc[:self.n] = np.zeros_like(self.n)
        mujoco_py.functions.mj_inverse(self.sim.model, self.gravity_comp_sim.data)
        comp = self.gravity_comp_sim.data.qfrc_inverse[:self.n].copy()
        return comp

    def gravity_compen2(self):
        # gravity compensation for 7 joints
        Jp_shape = (3, 41)
        comp = np.zeros((7,))
        for body, mass in zip(self.name_bodies, self.mass_links):
            Jp = self.sim.data.get_body_jacp(body).reshape(Jp_shape)
            comp = comp - np.dot(Jp.T, self.sim.model.opt.gravity * mass)
        return comp

    def inverse_kinematics(self, pose, seed=None):
        # todo error
        if seed is None:
            seed = self.q
        print('seed:', seed)
        result = self.ik.get_ik(seed,
                                pose[0], pose[1], pose[2],  # X, Y, Z
                                pose[4], pose[5], pose[6], pose[3])  # QX, QY, QZ, QW
        # self.ik_solver.CartToJnt()
        print("ik result:", result)
        return np.array(result) if result is not None else None

    def view_frame(self, pose):
        self.viewer.add_marker(pos=pose[:3],  # position of the arrow
                               size=np.array([0.005, 0.005, 0.05]),  # size of the arrow
                               mat=rot.quat2mat(pose[3:]),  # orientation as a matrix
                               rgba=np.array([1., 0., 0., .5]),  # color of the arrow
                               type=const.GEOM_ARROW,
                               # label=str('GEOM_ARROW')
                               )

    def view_frame3(self, body_name, length=0.1):
        x = self.sim.data.get_body_xpos(body_name)
        q = self.sim.data.get_body_xquat(body_name)
        R = rot.quat2mat(q)
        R_list = [R[:, [1, 2, 0]], R[:, [2, 0, 1]], R]
        RGBA = np.concatenate([np.identity(3), 0.8 * np.ones([3, 1])], axis=1)
        for i, r in enumerate(R_list):
            # add an offset at position, since cylinder starts from the center, and the height is 2 * length
            p = r @ np.array([0, 0, length]) + x
            self.viewer.add_marker(pos=p,  # position of the arrow
                                   size=np.array([0.005, 0.005, length]),  # size of the arrow
                                   mat=r,  # orientation as a matrix
                                   rgba=RGBA[i, :],  # color of the arrow
                                   type=const.GEOM_CYLINDER,  #
                                   # label=str('GEOM_ARROW')
                                   )

    def view_vector(self, pos, F, rgba=np.array([1., 0., 0., .5])):
        Fx_norm = np.linalg.norm(F[:3]) + 1e-7
        F1 = F[:3] / Fx_norm

        # F1x = np.zeros(3)
        # F1y = np.zeros(3)
        if np.abs(F1[0]) > np.abs(F1[1]):
            F1x = np.array([-F[2], 0, F1[0]])
        else:
            F1x = np.array([0, -F1[2], F1[1]])
        F1x = F1x / (np.linalg.norm(F1x) + 1e-10)
        F1y = np.cross(F1, F1x)
        F1y = F1y / (np.linalg.norm(F1y) + 1e-10)
        R = np.concatenate([F1x, F1y, F1]).reshape(3, 3).T

        length = Fx_norm / 1.
        self.viewer.add_marker(pos=pos,  # position of the arrow
                               size=np.array([0.005, 0.005, length]),  # size of the arrow
                               mat=R,  # orientation as a matrix
                               rgba=rgba,  # color of the arrow
                               type=const.GEOM_ARROW,
                               label=''  # make it empty, otherwise the viewer of contact force will be garbled
                               )

    def trans_force2world(self, pos, R, F):
        """
        Transformation of force and torque from a frame to world frame
        :param pos: position of the frame wrt world
        :param R:  rotation matrix
        :param F:  force and torque (6, )
        :return: force and torque in world. (6, )
        """
        S = np.array([[0, -pos[2], pos[1]],
                      [pos[2], 0, -pos[0]],
                      [-pos[1], pos[0], 0]])
        T1 = np.concatenate([R, np.zeros((3, 3))], axis=1)
        T2 = np.concatenate([S, R], axis=1)
        T = np.concatenate([T1, T2])
        return T @ F

    def contact_view(self):
        self.reset_contact_force()
        exploration_contact = [[], [], [], []]
        if self.sim.data.ncon:
            c_array = np.zeros(6, dtype=np.float64)

            for i in range(self.sim.data.ncon):
                c = self.sim.data.contact[i]
                # get the contact force in contact frame, which is defined by c.pos and c.frame
                mujoco_py.functions.mj_contactForce(self.sim.model, self.sim.data, i, c_array)
                # if np.linalg.norm(c_array[:3]) > 10:
                exclude_geom = ['grasping_table', 'floor']

                c_geom_name = [self.sim.model.geom_id2name(c.geom1), self.sim.model.geom_id2name(c.geom2)]
                # contact_name.append(c_geom_name)
                pos = c.pos
                # the contact frame axes are in the rows of the rotation matrix
                # https://roboti.us/forum/index.php?threads/something-wrong-when-return-contact-frame-and-contact-force.3348/
                ori = c.frame.reshape(3, 3).T

                # transfer it to world frame
                F = self.trans_force2world(pos, ori, c_array)
                for j in self.hand_contact_force:
                    if j == c_geom_name[0]:
                        self.hand_contact_force[j] -= F[:3]  # it's the force that gemo1 applies on gemo2
                    if j == c_geom_name[1]:
                        self.hand_contact_force[j] += F[:3]

                # if len(c_geom_name[0]) >= self.exploration_obj_str_num[0]:
                #     if c_geom_name[0][:self.exploration_obj_str_num[0]] in self.obj_explored and c_geom_name[
                #         1] in self.finger_tip:
                #         k = self.finger_tip.index(c_geom_name[1])
                #         exploration_contact[k].append(pos)
                # exploration_contact.append(np.concatenate([pos, -F]))
                if len(c_geom_name[1]) >= self.exploration_obj_str_num[0] and c_geom_name[0] in self.finger_tip:
                    if c_geom_name[1][:self.exploration_obj_str_num[0]] in self.obj_explored:
                        k = self.finger_tip.index(c_geom_name[0])
                        exploration_contact[k].append(np.concatenate([pos, F]))
                        # exploration_contact.append(np.concatenate([pos, F]))

                name_intersection = [j for j in c_geom_name if j in self.obj_explored or j in self.hand_contact_force]
                # if name_intersection:
                if 1:
                    # visualize the force (3,)
                    self.view_vector(pos, F)
                    if self.print_contact:
                        print('contact', i,
                              # " dist=", c.dist,
                              "   gemo1:", self.sim.model.geom_id2name(c.geom1),
                              "    gemo2:", self.sim.model.geom_id2name(c.geom2),
                              "    force/torque:", F[:3],  # it's the force that gemo1 applies on gemo2
                              "    pos:", pos,
                              # "    ori:", quaternion.as_float_array(quaternion.from_rotation_matrix(ori))
                              )

            for i, data in enumerate(exploration_contact):
                if data != []:
                    if len(data) == 1:
                        F_mean = data[0][0:6]
                    else:
                        all_F = np.vstack(data)

                        F_mean = np.mean(all_F[:, 0:6], axis=0)
                    self.exploration_contact[i] = F_mean
                else:
                    self.exploration_contact[i] = []

            # self.exploration_contact = exploration_contact

            for i, j in enumerate(self.hand_contact_force):
                self.hand_contact_force_norm[i] = np.linalg.norm(self.hand_contact_force[j])
        # else:
        #     # pass
        #     self.exploration_contact = [[], [], [], []]

        self.hand_contact = self.moving_average_filter()

    def moving_average_filter(self, size=100):
        self.hand_contact_force_norm_filter = np.concatenate(
            [self.hand_contact_force_norm_filter, self.hand_contact_force_norm.reshape(1, -1)])
        if self.hand_contact_force_norm_filter.shape[0] > size:
            self.hand_contact_force_norm_filter = np.delete(self.hand_contact_force_norm_filter, 0, 0)

        return np.mean(self.hand_contact_force_norm_filter, axis=0)

    def reset_contact_force(self):
        self.hand_contact_force_norm = np.zeros([5])
        for i in self.hand_contact_force:
            self.hand_contact_force[i] = np.array([0, 0, 0.])

    def sin_test(self, delta_z=0.2, T=20.):
        # sin movement test, z = A*sin(w*t)
        x0 = np.copy(self.x)
        print(x0)
        t = 0
        t0 = time.time()
        xd = x0
        while t < 100:
            t = time.time() - t0
            xd[2] += 0.2 * np.sin(2 * np.pi / T * t)
            self.move_to_pose()
        return True

    def test_contact(self):
        # without control, to see the contact viewer
        self.send_torque(np.zeros(self.n))

    def move_to_pose(self, pose, d_pose=np.zeros(6), k=None, t_duration=1., eps=1e-3,
                     control_type=CtrlType.IMPEDANCE_OP_SPACE):
        """
        Only control the iiwa
        move to desired pose of end-effector in Cartesian space
        :param k: kpx, kpv, krx, krv stiffness and damping parameter of impedance control
        :param pose: desired pose [x, y, z, qw, qx, qy, qz]
        :param d_pose:  desired pose vel
        :param t_duration: duration time for the motion
        :param eps:
        :param control_type: default by impedance control
        :return:
        """
        if k is None:
            kp = np.array([1, 0.3])
            kd = np.sqrt(kp) * 2
        if control_type == control_type == CtrlType.IMPEDANCE_OP_SPACE:
            Fx = kp[0] * (pose[:3] - self.x[:3]) + kd[0] * (d_pose[:3] - self.dx[:3])
            q = self.x[3:]  # [w x y z]
            qd = pose[3:]
            d_theta = (quaternion.from_float_array(qd) * (quaternion.from_float_array(q)).conjugate()).log() * 2
            d_theta = quaternion.as_float_array(d_theta)[1:]
            Fr = kp[1] * d_theta + kd[1] * (d_pose[3:] - self.dx[3:])
            F = np.concatenate([Fx, Fr])
            J = self.J
            impedance_acc_des = J.T.dot(np.linalg.solve(J.dot(J.T) + 1e-6 * np.eye(6), F))

            # Add stiffness and damping in the null space of the the Jacobian
            nominal_qpos = np.zeros(7)
            null_space_damping = 1
            null_space_stiffness = 1
            projection_matrix = J.T.dot(np.linalg.solve(J.dot(J.T), J))
            projection_matrix = np.eye(projection_matrix.shape[0]) - projection_matrix
            null_space_control = -null_space_damping * self.sim.data.qvel[:self.start_hand_joint]
            null_space_control += -null_space_stiffness * (
                    self.sim.data.qpos[:self.start_hand_joint] - nominal_qpos)
            impedance_acc_des += projection_matrix.dot(null_space_control)

            # Compute torques via inverse dynamics.
            acc_des = np.zeros(self.n)
            acc_des[:self.start_hand_joint] = impedance_acc_des
            self.sim.data.qacc[:self.n] = acc_des
            mujoco_py.functions.mj_inverse(self.sim.model, self.sim.data)
            id_torque = self.sim.data.qfrc_inverse[:self.n].copy()
            self.send_torque(id_torque)

        # if control_type == CtrlType.INV_KINEMASTICS:
        #     q = self.inverse_kinematics(pose)
        #     self.move_to_joints(q, t_duration=t_duration)

    # def move_hand(self, qd=None, kp=None, t_duration=5.):
    #     # move hand joints by impedance control
    #     qh =

    def move(self, pose=None, qh=None, d_pose=np.zeros(6), t_duration=0.002):
        if pose is None:
            pose = self.x
        if qh is None:
            qh = self.qh
        kp = np.array([30, 30.])
        kd = np.sqrt(kp) * 2
        Fx = kp[0] * (pose[:3] - self.x[:3]) + kd[0] * (d_pose[:3] - self.dx[:3])
        q = self.x[3:]  # [w x y z]
        qd = pose[3:]
        d_theta = (quaternion.from_float_array(qd) * (quaternion.from_float_array(q)).conjugate()).log() * 2
        d_theta = quaternion.as_float_array(d_theta)[1:]
        Fr = kp[1] * d_theta + kd[1] * (d_pose[3:] - self.dx[3:6])
        F = np.concatenate([Fx, Fr])
        J = self.J
        impedance_acc_des = J.T.dot(np.linalg.solve(J.dot(J.T) + 1e-6 * np.eye(6), F))

        # Add stiffness and damping in the null space of the the Jacobian
        nominal_qpos = np.zeros(7)
        null_space_damping = 1
        null_space_stiffness = 1
        projection_matrix = J.T.dot(np.linalg.solve(J.dot(J.T), J))
        projection_matrix = np.eye(projection_matrix.shape[0]) - projection_matrix
        null_space_control = -null_space_damping * self.sim.data.qvel[:self.start_hand_joint]
        null_space_control += -null_space_stiffness * (
                self.sim.data.qpos[:self.start_hand_joint] - nominal_qpos)
        impedance_acc_des += projection_matrix.dot(null_space_control)

        # hand joint impedance
        traj_q = qh.reshape(1, -1)
        traj_dq = (self.qh - self.q_last[self.start_hand_joint:self.n]).reshape(1, -1) / t_duration
        qpos_ref, qvel_ref = traj_q[0, :], traj_dq[0, :]

        error_q = qpos_ref - self.qh
        error_dq = qvel_ref - self.dqh
        error_dq = 0 - self.dqh
        kp = np.ones(16) * 0.2
        kd = 2 * np.sqrt(kp) * 0.01
        qacc_des = kp * error_q + kd * error_dq
        self.sim.data.qacc[:self.n] = np.concatenate([impedance_acc_des.copy(), np.zeros(16)])
        # self.sim.data.qpos[self.start_hand_joint:self.n] = qh.copy()
        # self.sim.data.qvel[self.start_hand_joint:self.n] = np.zeros(16)
        mujoco_py.functions.mj_inverse(self.sim.model, self.sim.data)
        id_torque = self.sim.data.qfrc_inverse[:self.n].copy()
        id_torque[self.start_hand_joint:] = qacc_des
        self.send_torque(id_torque)

    def hand_move_torque(self, qh=None, dqh=None):
        if qh is None:
            qh = np.zeros(16)
            qh[12] = 0.5
        if dqh is None:
            dqh = np.zeros(16)
        error_q = qh - self.qh
        error_dq = dqh - self.dqh
        u = np.zeros(16)

        kp = np.ones(16) * 0.4 * 0.3
        kd = 2 * np.sqrt(kp) * 0.01
        kd = 0.01
        # kd = np.ones(16) * 1
        qacc_des = kp * error_q + kd * error_dq + self.C_[self.start_hand_joint:]

        # print('vel', self.dqh[:4])
        # print('pos_error', error_q[:4])
        # print('control torque:', qacc_des[12:])

        u = np.concatenate([np.zeros(7), qacc_des])

        # self.send_torque(u)
        return qacc_des

    def left_hand_move(self, qh=None, dqh=None, u_add=None, scale=1, iiwa=False, pose=None):
        if qh is None:
            qh = np.zeros(16)
            qh[0] = 0.5
        if dqh is None:
            dqh = np.zeros(16)
        if u_add is None:
            u_add = np.zeros(16)
        error_q = qh - self.qh
        error_dq = dqh - self.dqh
        # u = np.zeros(16)
        kp = np.ones(16) * 0.4 * scale
        kd = 2 * np.sqrt(kp) * 0.01 * 0.5 * scale
        # kd = 0
        # kd = 0.01
        # kd = np.ones(16) * 1
        # qacc_des = kp * error_q + kd * error_dq + self.C_[self.start_hand_joint:] + u_add
        qacc_des = kp * error_q + kd * error_dq + self.C_[self.start_hand_joint:]
        # qacc_des = self.C_[self.start_hand_joint:] + u_add

        # print('vel', self.dqh[:4])
        # print('pos_error', error_q[:4])
        # print('control torque:', qacc_des[12:])
        if iiwa:
            iiwa_torque = self.iiwa_test(pose)
        else:
            iiwa_torque = np.zeros(7)
        u = np.concatenate([iiwa_torque, qacc_des])

        self.send_torque(u)

    def hand_move(self, qd=None):
        if qd is None:
            qd = np.zeros(16)
            qd[12] = 0.5

        u = np.concatenate([np.zeros(7), qd])
        # print(u)
        self.send_torque(u)

    def move_to_all_joints(self, qd=None, kp=None, t_duration=5.):
        """
        :param kp:
        :param qd:
        :param t_duration:
        :return:
        """
        if qd is None:
            qd = np.array([0, 0, 0, -np.pi / 2, 0, 0, 0])
            qd = np.concatenate([qd, np.zeros(16)])
        if t_duration < 0.0041:
            traj_q = qd.reshape(1, -1)
            traj_dq = (self.q_ - self.q_last).reshape(1, -1) / t_duration
            # traj_ddq = self.ddq_.reshape(1, -1)
            traj_ddq = np.zeros(self.n).reshape(1, -1)
        else:
            traj_q, traj_dq, traj_ddq = traj_joint_interpolation(qd, ti=self.sim.data.time, t_duration=t_duration,
                                                                 q_act=self.q_, repeat_goal=100,
                                                                 traj_profile=TrajectoryProfile.SPLINE3)
        for i in range(traj_q.shape[0]):
            # mujoco_py.functions.mj_inverse(self.sim.model, self.sim.data)
            qpos_ref, qvel_ref, qacc_ref = traj_q[i, :], traj_dq[i, :], traj_ddq[i, :]

            error_q = qpos_ref - self.q_
            error_dq = qvel_ref - self.dq_
            u = np.zeros(self.n)
            # kp = np.concatenate([np.ones(7) * 10., np.ones(16) * 1])
            # kd = 2 * np.sqrt(kp)
            kp = np.concatenate([np.ones(7) * 10., np.ones(16) * 10])
            kd = 2 * np.sqrt(kp)
            qacc_des = kp * error_q + kd * error_dq
            self.sim.data.qacc[:self.n] = qacc_des.copy()
            mujoco_py.functions.mj_inverse(self.sim.model, self.sim.data)
            id_torque = self.sim.data.qfrc_inverse[:self.n].copy()
            u = id_torque
            # print(u)
            self.send_torque(u)

    def move_to_joints(self, qd=None, t_duration=5., eps=1 * np.pi / 180, control_type=CtrlType.INV_DYNAMICS_JOINT_PD,
                       ):
        """
        move to desired joint position, by joint feedback control
        :param control_type: choose the control type
        :param qd: desired joint position
        :param t_duration: duration time for the motion
        :param eps: eps for joint error
        :return:
        """
        if qd is None:
            qd = np.array([0, 0, 0, -np.pi / 2, 0, np.pi / 2, 0])
            # self.qd = qd
        print(qd)
        # do interpolation for trajectories

        if t_duration < 0.004:
            traj_q = qd.reshape(1, -1)
            traj_dq = (self.q - q_last).reshape(1, -1) / t_duration
            traj_ddq = self.ddq.reshape(1, -1)
        else:
            traj_q, traj_dq, traj_ddq = traj_joint_interpolation(qd, ti=self.sim.data.time, t_duration=t_duration,
                                                                 q_act=self.q, repeat_goal=100,
                                                                 traj_profile=TrajectoryProfile.SPLINE3)
        # print(traj_q)
        error_q_int = 0

        Kp = np.eye(7)
        Kd = np.eye(7)
        Ki = np.zeros((7, 7))
        for i in range(7):
            Kp[i, i] = 10 * (1 - 0.15 * i) * 4
            Kd[i, i] = 1.2 * (Kp[i, i] / 2) ** 0.5

        Kd[6, 6] = Kd[6, 6] / 50

        for i in range(traj_q.shape[0]):
            qpos_ref, qvel_ref, qacc_ref = traj_q[i, :], traj_dq[i, :], traj_ddq[i, :]

            error_q = qpos_ref - self.q
            error_dq = qvel_ref - self.dq
            error_q_int = (error_q + error_q_int) * self.dt / 2 + error_q_int

            u = np.zeros(7)
            mujoco_py.functions.mj_inverse(self.sim.model, self.sim.data)

            if control_type == CtrlType.INDEP_JOINT_PID:
                u = Kp @ error_q + Kd @ error_dq + Ki @ error_q_int + self.sim.data.qfrc_inverse[:self.start_hand_joint]

            if control_type == CtrlType.INV_DYNAMICS_JOINT_PD:
                # v = qacc_ref + Kp @ error_q + Kd @  error_dq
                # compensation
                # u = self.M @ v + self.C + self.gravity_compen2()  # with full dynamics
                # print(u)

                kp = 10.
                kd = 2 * np.sqrt(kp)
                qacc_des = kp * error_q + kd * error_dq
                self.sim.data.qacc[:self.start_hand_joint] = qacc_des.copy()
                mujoco_py.functions.mj_inverse(self.sim.model, self.sim.data)
                id_torque = self.sim.data.qfrc_inverse[:self.start_hand_joint].copy()
                u = id_torque

            if self.log:
                self.log_x.append(self.x)
                self.log_q.append(self.q)
                self.log_qd.append(qpos_ref)
                self.log_dqd.append(qvel_ref)
                self.log_u.append(self.tau)
                self.log_ud.append(u)
                self.log_time.append(self.sim.data.time)

            self.send_torque(u)
            if self.viewer is not None:
                self.viewer.render()
        # print('joint trajectory done! \n joint error (in degree):')
        # print((qd - self.q) * 180 / np.pi)

    # def forward_kin(self, pose):
    #     mujoco_py.functions.mj_local2Global()
    def iiwa_Cartesion_impedance(self, pose, d_pose=None, kp=None):
        if d_pose is None:
            d_pose = np.zeros(6)
        if kp is None:
            kp = np.array([300, 200.])
        kd = np.sqrt(kp) * 2 * 2. * 3
        pos_error = pose[:3] - self.x[:3]
        vel_error = d_pose[:3] - self.dx[:3]
        Fx = kp[0] * (pose[:3] - self.x[:3]) + kd[0] * (d_pose[:3] - self.dx[:3])
        q = self.x[3:]  # [w x y z]
        qd = pose[3:]
        d_theta = (quaternion.from_float_array(qd) * (quaternion.from_float_array(q)).conjugate()).log() * 2
        d_theta = quaternion.as_float_array(d_theta)[1:]
        Fr = kp[1] * d_theta + kd[1] * (d_pose[3:] - self.dx[3:6])
        F = np.concatenate([Fx, Fr])
        J = self.J
        # impedance_acc_des0 = J.T.dot(np.linalg.solve(J.dot(J.T) + 1e-10 * np.eye(6), F))
        impedance_acc_des1 = J.T @ F
        return impedance_acc_des1



    def iiwa_test(self, pose, d_pose=None):
        if d_pose is None:
            d_pose = np.zeros(6)
        kp = np.array([300, 200.])
        kd = np.sqrt(kp) * 2 * 2. * 3
        # kd = np.sqrt(kp) * 1
        pos_error = pose[:3] - self.x[:3]
        vel_error = d_pose[:3] - self.dx[:3]
        Fx = kp[0] * (pose[:3] - self.x[:3]) + kd[0] * (d_pose[:3] - self.dx[:3])
        q = self.x[3:]  # [w x y z]
        qd = pose[3:]
        d_theta = (quaternion.from_float_array(qd) * (quaternion.from_float_array(q)).conjugate()).log() * 2
        d_theta = quaternion.as_float_array(d_theta)[1:]
        Fr = kp[1] * d_theta + kd[1] * (d_pose[3:] - self.dx[3:6])
        F = np.concatenate([Fx, Fr])
        J = self.J
        impedance_acc_des0 = J.T.dot(np.linalg.solve(J.dot(J.T) + 1e-10 * np.eye(6), F))
        impedance_acc_des1 = J.T @ F

        # Add stiffness and damping in the null space of the the Jacobian
        nominal_qpos = np.zeros(7)
        null_space_damping = 0.1 * 10
        null_space_stiffness = 10 * 5
        projection_matrix = J.T.dot(np.linalg.solve(J.dot(J.T) + 1e-10 * np.eye(6), J))
        projection_matrix = np.eye(projection_matrix.shape[0]) - projection_matrix
        null_space_control = -null_space_damping * self.sim.data.qvel[:self.start_hand_joint]
        null_space_control += -null_space_stiffness * (
                self.sim.data.qpos[:self.start_hand_joint] - nominal_qpos)
        tau_null = projection_matrix.dot(null_space_control)
        impedance_acc_des = impedance_acc_des1 + tau_null

        # self.send_torque(impedance_acc_des + self.C)
        return impedance_acc_des + self.C

    def iiwa_hand_move(self, pose, qh, t=.001):

        if t > 0.002:
            interp_q = np.concatenate([pose[:3], qh])
            interp_q0 = np.concatenate([self.x[:3], self.qh])
            traj_q, traj_dq, _ = traj_joint_interpolation(interp_q, ti=self.sim.data.time, t_duration=t,
                                                          q_act=interp_q0, repeat_goal=10,
                                                          traj_profile=TrajectoryProfile.SPLINE3)
            sample_num = traj_q.shape[0]
            ori = slerp(self.x[3:], pose[3:], np.array(range(sample_num), dtype=np.float) / (sample_num - 1))
            # print(ori.shape)
            for i in range(sample_num):
                qpos_ref, qvel_ref = traj_q[i, :], traj_dq[i, :]
                p1 = np.concatenate([qpos_ref[:3], ori[i, :]])
                dp1 = np.concatenate([qvel_ref[:3], np.zeros(3)])
                self.iiwa_hand_go(p1, qpos_ref[3:], d_pose=dp1, dqh=qvel_ref[3:])
        else:
            self.iiwa_hand_go(pose, qh)

    def iiwa_hand_go(self, pose, qh, d_pose=None, dqh=None):

        iiwa_torque = self.iiwa_test(pose, d_pose=d_pose)
        hand_torque = self.hand_move_torque(qh=qh, dqh=dqh)
        u = np.concatenate([iiwa_torque, hand_torque])
        self.send_torque(u)

    def move_xyz(self, xyz, qh1, t):
        xd = np.array([4.80256079e-01, -4.60920270e-04, 1.28706708e+00, 0, np.sqrt(2) / 2, 0, np.sqrt(2) / 2])
        xd[:3] += xyz
        t0 = time.time()
        while True:
            t1 = time.time() - t0
            self.iiwa_hand_move(xd, qh1)
            if t1 > t:
                break

    def moveto_attractor(self, xd, qh, dt=0.002, couple_hand=True):
        """
        move to the position attractor by the coupled DS
        :param xd: (7,) position and quaternion[x, q]
        :param qh: (16,)  joint angles
        :return:
        """
        dx, w, d_theta = self.coupled_DS(xd[:3], xd[3:], qh, couple_hand=couple_hand)
        # print(dx, w, d_theta)
        d_pose = np.concatenate([dx, w])
        # integral
        xd_, qh_ = self.integral_from_vel(dx, w, d_theta, dt)

        # print(xd_[:3] - xd[:3])
        # print(rot.ori_dis(xd_[3:], xd[3:]) * 180/np.pi)
        self.iiwa_hand_go(xd_, qh_, d_pose=None)

    def coupled_DS(self, xd, qd, theta_d, couple_hand=True):
        """

        :param xd:  (3,) position of end-effector
        :param qd:   (4,) quaternion of --
        :param theta_d: (16, ) joint angles of hand
        :return: dx, w, \dot{theta} : velocities (3,), (3,), (16,)
        """
        x, q = self.x[:3], self.x[3:]
        theta = self.qh
        if q[0] < 0:
            q = -q
        if qd[0] < 0:
            qd = -qd
        q = quaternion.from_float_array(q)
        qd = quaternion.from_float_array(qd)

        # parameters for DS
        a = 10
        b = 100
        c = 70
        lambda_1 = 10
        lambda_2 = 50
        lambda_2 = 150

        # position DS
        a = a / (np.linalg.norm(x - xd) + 0.002)
        dx = -1 * a * (x - xd)

        # orientation DS
        dis = np.linalg.norm(x - xd)
        if dis < 0.005:
            dis = 0
        t = np.exp(- lambda_1 * dis)
        qd_ = quaternion.slerp(q, qd, 0, 1, t)
        eps = 1 / 180. * np.pi
        b = b / (quaternion.rotation_intrinsic_distance(q, qd_) + eps)
        w = - b * (q * qd_.conj()).log().vec

        # hand DS
        theta_d_ = theta + (theta_d - theta) * np.exp(- lambda_2 * dis)
        # print(theta_d_)
        if couple_hand is False:
            theta_d_ = theta_d
        c = c / (np.abs(theta - theta_d_) + eps)  # for each joint should take different c?

        d_theta = - c * (theta - theta_d_)

        return dx, w, d_theta

    def integral_from_vel(self, dx, w, d_theta, dt):
        """

        :param dx:
        :param w:
        :param d_theta:
        :param dt:
        :return: [x, q], qh
        """
        x = self.x[:3] + dx * dt
        q = quaternion.from_rotation_vector(w * dt) * quaternion.from_float_array(self.x[3:])
        q = quaternion.as_float_array(q)

        qh = self.qh + d_theta * dt

        return np.concatenate([x, q]), qh

    # todo, the simulation might be unstable if applies a force
    def apply_force(self, force, body, torque=None, point=None):
        if torque is None:
            torque = np.zeros(3)
        if point is None:
            point = np.zeros(3)
        body_id = self.sim.model.body_name2id(body)
        # print(body_id)
        qfrc = np.zeros(self.sim.model.nv, dtype=np.float64)
        mujoco_py.functions.mj_applyFT(self.sim.model, self.sim.data, force, torque, point, body_id, qfrc)
        # print(qfrc)
        print(self.sim.data.qfrc_applied)
        self.sim.data.qfrc_applied[:] = qfrc

    def clear_force(self):
        self.sim.data.qfrc_applied[:] = 0

    @property
    def q(self):
        """
        iiwa joint angles
        return: (7, )
        """
        return self.sim.data.qpos[self.iiwa_joint_id]

    @property
    def qh(self):
        """
        hand angles: thumb - index - middle - ring
        :return: (16, )
        """
        return self.sim.data.qpos[self.hand_joint_id]

    @property
    def q_(self):
        """
        hand angles
        :return: (23, )
        """
        return self.sim.data.qpos[self.iiwa_hand_joint_id]

    @property
    def dq(self):
        """
        joint angular velocities
        return: (7, )
        """
        return self.sim.data.qvel[self.iiwa_joint_id]

    @property
    def dq_(self):
        """
        joint angular velocities
        return: (23, )
        """
        return self.sim.data.qvel[self.iiwa_hand_joint_id]

    @property
    def dqh(self):
        """
        hand joint angular velocities
        return: (7, )
        """
        return self.sim.data.qvel[self.hand_joint_id]

    @property
    def ddqh(self):
        """
        hand joint angular velocities
        return: (7, )
        """
        return self.sim.data.qacc[self.hand_joint_id]

    @property
    def ddq(self):
        """
        joint angular acceleration
        return: (7, )
        """
        return self.sim.data.qacc[:self.iiwa_joint_id]

    @property
    def ddq_(self):
        """
        joint angular acceleration
        return: (23, )
        """
        return self.sim.data.qacc[self.iiwa_hand_joint_id]

    @property
    def xh(self):
        """
        hand tip pose:  index - middle - ring - thumb
        :return: (4, 7) position and orientation of finger tips
        """
        x1 = self.sim.data.get_site_xpos("thumb_site")
        q1 = rot.mat2quat(self.sim.data.get_site_xmat("thumb_site"))
        x2 = self.sim.data.get_site_xpos("index_site")
        q2 = rot.mat2quat(self.sim.data.get_site_xmat("index_site"))
        x3 = self.sim.data.get_site_xpos("middle_site")
        q3 = rot.mat2quat(self.sim.data.get_site_xmat("middle_site"))
        x4 = self.sim.data.get_site_xpos("ring_site")
        q4 = rot.mat2quat(self.sim.data.get_site_xmat("ring_site"))

        p = np.concatenate([x1, q1, x2, q2, x3, q3, x4, q4])
        return p.reshape(4, 7)


    @property
    def xh_site1(self):
        """
        hand tip pose:  index - middle - ring - thumb
        :return: (4, 7) position and orientation of finger tips
        """
        x1 = self.sim.data.get_site_xpos("thumb_site1")
        q1 = rot.mat2quat(self.sim.data.get_site_xmat("thumb_site1"))
        x2 = self.sim.data.get_site_xpos("index_site1")
        q2 = rot.mat2quat(self.sim.data.get_site_xmat("index_site1"))
        x3 = self.sim.data.get_site_xpos("middle_site1")
        q3 = rot.mat2quat(self.sim.data.get_site_xmat("middle_site1"))
        x4 = self.sim.data.get_site_xpos("ring_site1")
        q4 = rot.mat2quat(self.sim.data.get_site_xmat("ring_site1"))

        p = np.concatenate([x1, q1, x2, q2, x3, q3, x4, q4])
        return p.reshape(4, 7)


    @property
    def x(self):
        """
        Cartesian position and orientation (quat) of the end-effector frame, TCP
            computed by mj_fwdPosition/mj_kinematics of mujoco
        The quaternion is in [w, x, y, z] order
        return: (7, )
        """
        xpos = self.sim.data.get_site_xpos(self.name_tcp)
        xR = self.sim.data.get_site_xmat(self.name_tcp)
        xquat = rot.mat2quat(xR)
        # print(xpos)
        # print(xquat)
        return np.concatenate([xpos, xquat])

    @property
    def p(self):
        """
        :return: transformation matrix (4, 4)
        """
        pos = self.x[:3].reshape(-1, 1)
        quat = self.x[3:]
        R = self.qua2Mat(quat)
        return np.concatenate([np.concatenate([R, pos], axis=1), np.array([[0., 0, 0, 1]])])

    @property
    def x_obj(self):
        """
        :return: objects poses by list
        """  # print(joint_id)
        poses = []
        for i in self.obj_explored:
            poses.append(np.concatenate([self.sim.data.get_body_xpos(i), self.sim.data.get_body_xquat(i)]))
        return poses

    @property
    def dx(self):
        """
        Cartesian velocities of the end-effector frame, TCP
            computed by jacbian @ qvel
        return: (6, )
        """
        dx_p = self.sim.data.get_site_xvelp(self.name_tcp)
        dx_r = self.sim.data.get_site_xvelr(self.name_tcp)
        return np.concatenate([dx_p, dx_r])

    @property
    def J(self):
        """
        Jacobian of the end-effector
        :return: (6, 7) numpy array
        """
        Jp_shape = (3, self.sim.model.nv)
        jacp = self.sim.data.get_site_jacp(self.name_tcp).reshape(Jp_shape)
        jacr = self.sim.data.get_site_jacr(self.name_tcp).reshape(Jp_shape)
        return np.vstack((jacp[:3, :self.start_hand_joint], jacr[:3, :self.start_hand_joint]))

    @property
    def J_site(self):
        """
        Jacobian of the fingertips by using site
        :return: dict for all site
        """
        Jp_shape = (3, 4)
        J = {}
        for i, site_name in enumerate(self.site):
            jacp = self.sim.data.get_site_jacp(site_name).reshape(3, -1)
            if jacp.shape[1] > 23:
                jacp = jacp[:, :23]  # this is wrong, iiwa joint, free joint, hand joint
            # jacp = self.sim.data.get_site_jacp(site_name).reshape(3, 23)[:, 7 + i * 4:11 + i * 4]
            # jacr = self.sim.data.get_site_jacr(site_name).reshape(3, 23)[:, 7 + i * 4:11 + i * 4]
            J[site_name] = jacp
        return J

    ## dynamical-relevant property
    @property
    def tau(self):
        """
        actuator torque
        :return:
        """
        return self.sim.data.actuator_force[self.iiwa_joint_id]

    @property
    def tau_(self):
        """
        actuator torque
        :return:
        """
        return self.sim.data.actuator_force[self.iiwa_hand_joint_id]

    @property
    def tau_h(self):
        """
        actuator torque
        :return:
        """
        return self.sim.data.actuator_force[self.hand_joint_id]

    @property
    def M(self):
        """
        get inertia matrix of the iiwa
        :return:
        """
        tmp = np.zeros(self.nv * self.nv)
        mujoco_py.functions.mj_fullM(self.sim.model, tmp, self.sim.data.qM)
        M = tmp.reshape(self.nv, self.nv)

        return M[:7, :7]

    @property
    def M_(self):
        """
        get inertia matrix for all bodies
        :return:
        """
        tmp = np.zeros(self.nv * self.nv)
        mujoco_py.functions.mj_fullM(self.sim.model, tmp, self.sim.data.qM)
        M_ = tmp.reshape(self.nv, self.nv)

        return M_[:self.n, :self.n]

    @property
    def C(self):
        """
        C(qpos,qvel), Coriolis
        :return:
        """
        return self.sim.data.qfrc_bias[:self.start_hand_joint]

    @property
    def C_(self):
        """
        C(qpos,qvel), Coriolis
        :return:
        """
        return self.sim.data.qfrc_bias[:self.n]


def switch_qua(q):
    """
    :param q: quaternion as [x y z w], numpy array
    :return:  quaternion as [w x y z]
    """
    return q[[3, 0, 1, 2]]


def traj_joint_interpolation(qd, ti, t_duration=1., dt=0.002, q_act=None,
                             traj_profile=TrajectoryProfile.SPLINE3, repeat_goal=0):
    """
    Joint trajectory generation with different methods.
    :param repeat_goal: repeat the goal point to reach a precise joint positions
    :param qd: joint space desired final point
    :param t_duration: total time of travel
    :param n: number of time steps
    :param ti: initial time
    :param dt: time step
    :param q_act: current joint position
    :param traj_profile: type of trajectory 'step', 'spline3', 'spline5', 'trapvel'
    :return:
    """
    n_timesteps = int(t_duration / dt)
    N = qd.shape[0]
    traj_q = np.zeros((n_timesteps, N))
    traj_dq = np.zeros((n_timesteps, N))
    traj_ddq = np.zeros((n_timesteps, N))
    t_array = np.linspace(ti, t_duration + ti, n_timesteps)
    # tf = ti + t_duration

    if traj_profile == TrajectoryProfile.SPLINE3:
        T = t_duration
        qvel0 = np.zeros((N,))
        qvelf = np.zeros((N,))
        Q = np.array([q_act, qvel0, qd, qvelf])
        A_inv = np.linalg.inv(np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0],
                                        [1, T, T ** 2, T ** 3],
                                        [0, 1, 2 * T, 3 * T ** 2]]))
        coeffs = np.dot(A_inv, Q)
        for i, t in enumerate(t_array):
            q_ref = np.dot(np.array([1, (t - ti), (t - ti) ** 2, (t - ti) ** 3]), coeffs)
            qvel_ref = np.dot(np.array([0, 1, 2 * (t - ti), 3 * (t - ti) ** 2]), coeffs)
            qacc_ref = np.dot(np.array([0, 0, 2, 6 * (t - ti)]), coeffs)
            # yield q_ref, qvel_ref, qacc_ref
            traj_q[i, :] = q_ref
            traj_dq[i, :] = qvel_ref
            traj_ddq[i, :] = qacc_ref

    if traj_profile == TrajectoryProfile.SPLINE5:
        T = t_duration
        qvel0 = np.zeros((7,))
        qacc0 = np.zeros((7,))
        qvelf = np.zeros((7,))
        qaccf = np.zeros((7,))
        Q = np.array([q_act, qvel0, qacc0, qd, qvelf, qaccf])
        A_inv = np.linalg.inv(np.array([[1, 0, 0, 0, 0, 0],
                                        [0, 1, 0, 0, 0, 0],
                                        [0, 0, 2, 0, 0, 0],
                                        [1, T, T ** 2, T ** 3, T ** 4, T ** 5],
                                        [0, 1, 2 * T, 3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                                        [0, 0, 2, 6 * T, 12 * T ** 2, 20 * T ** 3]]))
        coeffs = np.dot(A_inv, Q)
        for i, t in enumerate(t_array):
            q_ref = np.dot(np.array([1, (t - ti), (t - ti) ** 2, (t - ti) ** 3, (t - ti) ** 4, (t - ti) ** 5]),
                           coeffs)
            qvel_ref = np.dot(
                np.array([0, 1, 2 * (t - ti), 3 * (t - ti) ** 2, 4 * (t - ti) ** 3, 5 * (t - ti) ** 4]), coeffs)
            qacc_ref = np.dot(np.array([0, 0, 2, 6 * (t - ti), 12 * (t - ti) ** 2, 20 * (t - ti) ** 3]), coeffs)
            # yield q_ref, qvel_ref, qacc_ref
            traj_q[i, :] = q_ref
            traj_dq[i, :] = qvel_ref
            traj_ddq[i, :] = qacc_ref
    if repeat_goal == 0:
        return traj_q, traj_dq, traj_ddq
    else:
        traj_q = np.concatenate([traj_q, np.repeat(qd.reshape(1, -1), repeat_goal, axis=0)])
        traj_dq = np.concatenate([traj_dq, np.repeat(np.zeros((1, N)), repeat_goal, axis=0)])
        traj_ddq = np.concatenate([traj_ddq, np.repeat(np.zeros((1, N)), repeat_goal, axis=0)])
        return traj_q, traj_dq, traj_ddq


#
# class TrajGen:
#
#     def __init__(self):
#         self.extra_points = 0
#         self.iterator = None
#
#     def next(self):
#         assert (self.iterator is not None)
#         return next(self.iterator)
#
#
# class TrajectoryJoint(TrajGen):
#
#     def __init__(self, qd, ti, t_duration=1, dt=0.002, q_act=np.zeros((7,)), traj_profile=None):
#         super(TrajectoryJoint, self).__init__()
#         self.iterator = self._traj_implementation(qd, ti, t_duration, dt, q_act, traj_profile)
#
#     def _traj_implementation(self, qd, ti, t_duration=1, dt=0.002, q_act=np.zeros((7,)), traj_profile=None):
#         """
#         Joint trajectory generation with different methods.
#         :param qd: joint space desired final point
#         :param t_duration: total time of travel
#         :param n: number of time steps
#         :param ti: initial time
#         :param dt: time step
#         :param q_act: current joint position
#         :param traj_profile: type of trajectory 'step', 'spline3', 'spline5', 'trapvel'
#         :return:
#         """
#         n_timesteps = int(t_duration / dt)
#         traj_q = np.zeros((n_timesteps, 7))
#         traj_dq = np.zeros((n_timesteps, 7))
#         traj_ddq = np.zeros((n_timesteps, 7))
#         time = np.linspace(ti, t_duration + ti, n_timesteps)
#         # tf = ti + t_duration
#
#         if traj_profile == TrajectoryProfile.SPLINE3:
#             T = t_duration
#             qvel0 = np.zeros((7,))
#             qvelf = np.zeros((7,))
#             Q = np.array([q_act, qvel0, qd, qvelf])
#             A_inv = np.linalg.inv(np.array([[1, 0, 0, 0],
#                                             [0, 1, 0, 0],
#                                             [1, T, T ** 2, T ** 3],
#                                             [0, 1, 2 * T, 3 * T ** 2]]))
#             coeffs = np.dot(A_inv, Q)
#             for i, t in enumerate(time):
#                 q_ref = np.dot(np.array([1, (t - ti), (t - ti) ** 2, (t - ti) ** 3]), coeffs)
#                 qvel_ref = np.dot(np.array([0, 1, 2 * (t - ti), 3 * (t - ti) ** 2]), coeffs)
#                 qacc_ref = np.dot(np.array([0, 0, 2, 6 * (t - ti)]), coeffs)
#                 # yield q_ref, qvel_ref, qacc_ref
#                 traj_q[i,:] = q_ref
#                 traj_dq[i,:] = qvel_ref
#                 traj_ddq[i,:] = qacc_ref
#
#         if traj_profile == TrajectoryProfile.SPLINE5:
#             T = t_duration
#             qvel0 = np.zeros((7,))
#             qacc0 = np.zeros((7,))
#             qvelf = np.zeros((7,))
#             qaccf = np.zeros((7,))
#             Q = np.array([q_act, qvel0, qacc0, qd, qvelf, qaccf])
#             A_inv = np.linalg.inv(np.array([[1, 0, 0, 0, 0, 0],
#                                             [0, 1, 0, 0, 0, 0],
#                                             [0, 0, 2, 0, 0, 0],
#                                             [1, T, T ** 2, T ** 3, T ** 4, T ** 5],
#                                             [0, 1, 2 * T, 3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
#                                             [0, 0, 2, 6 * T, 12 * T ** 2, 20 * T ** 3]]))
#             coeffs = np.dot(A_inv, Q)
#             for i, t in enumerate(time):
#                 q_ref = np.dot(np.array([1, (t - ti), (t - ti) ** 2, (t - ti) ** 3, (t - ti) ** 4, (t - ti) ** 5]),
#                                coeffs)
#                 qvel_ref = np.dot(
#                     np.array([0, 1, 2 * (t - ti), 3 * (t - ti) ** 2, 4 * (t - ti) ** 3, 5 * (t - ti) ** 4]), coeffs)
#                 qacc_ref = np.dot(np.array([0, 0, 2, 6 * (t - ti), 12 * (t - ti) ** 2, 20 * (t - ti) ** 3]), coeffs)
#                 # yield q_ref, qvel_ref, qacc_ref
#                 traj_q[i, :] = q_ref
#                 traj_dq[i, :] = qvel_ref
#                 traj_ddq[i, :] = qacc_ref
#         # while True:
#         #     self.extra_points = self.extra_points + 1
#         #     yield q_ref, qvel_ref, qacc_ref
def slerp(v0, v1, t_array):
    """Spherical linear interpolation."""
    # from https://en.wikipedia.org/wiki/Slerp
    # >>> slerp([1,0,0,0], [0,0,0,1], np.arange(0, 1, 0.001))
    t_array = np.array(t_array)
    v0 = np.array(v0)
    v1 = np.array(v1)
    dot = np.sum(v0 * v1)

    if dot < 0.0:
        v1 = -v1
        dot = -dot

    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        result = v0[np.newaxis, :] + t_array[:, np.newaxis] * (v1 - v0)[np.newaxis, :]
        return (result.T / np.linalg.norm(result, axis=1)).T

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)

    theta = theta_0 * t_array
    sin_theta = np.sin(theta)

    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0[:, np.newaxis] * v0[np.newaxis, :]) + (s1[:, np.newaxis] * v1[np.newaxis, :])


def position_intepolation(x0, x1, t=2):
    """

    :param x0: (3,) robot position or (16,) hand joint
    :param x1: (3,) or (16,)
    :return: (n,3) or (n,16)
    """

# def motion_generation(p1, pose, vel=0.2, intepolation='linear', ):
#     # poses : (n,7) array, n: num of viapoints. [position, quaternion]
#     # print(self._x )
#     constant_ori = np.array([0, 1.0, 0, 0])
#     if self.x is None:
#         return 0
#     poses = np.concatenate([self.x.reshape(1, -1), pose], axis=0)  # add current points
#     keypoints_num = poses.shape[0]
#
#     path_length = 0
#     for i in range(keypoints_num - 1):
#         path_length += np.linalg.norm(poses[i, :3] - poses[i + 1, :3])
#     path_time = path_length / vel
#
#     sample_freq = 0.5  # 2s for each point
#     if not self._stop:
#         # position = np.zeros([3, keypoints_num-1])
#         # orientation = np.zeros([4, keypoints_num-1])
#         for i in range(keypoints_num - 1):
#             path_i = np.linalg.norm(poses[i, :3] - poses[i + 1, :3])
#             # print(path_i)
#             sample_num = int(path_i / vel * sample_freq + 1)
#             if sample_num < 2:
#                 sample_num = 2
#
#             if debug:
#                 print(path_i)
#                 rospy.loginfo(
#                     "start to go the " + str(i + 1) + "-th point: " + " x=" + str(poses[i + 1, 0]) + " y=" + str(
#                         poses[i + 1, 1])
#                     + " z=" + str(poses[i + 1, 2]) + " time: " + str(path_i / vel) + "s")
#             if intepolation == 'linear':
#                 pos = np.concatenate((np.linspace(poses[i, 0], poses[i + 1, 0], num=sample_num).reshape(-1, 1),
#                                       np.linspace(poses[i, 1], poses[i + 1, 1], num=sample_num).reshape(-1, 1),
#                                       np.linspace(poses[i, 2], poses[i + 1, 2], num=sample_num).reshape(-1, 1)),
#                                      axis=1)  # print
#             ori = self.slerp(poses[i, 3:], poses[i + 1, 3:],
#                              np.array(range(sample_num), dtype=np.float) / (sample_num - 1))
#             ori = np.repeat(constant_ori.reshape(1, -1), repeats=sample_num, axis=0)
#             delta_t = path_i / vel / (sample_num - 1)
#             target_pose = np.concatenate((pos, ori), axis=1)
#             self.move_to_frame(target_pose, path_i / vel)
#             # for j in range(sample_num):
#             #     target_x = np.concatenate((pos[j,:], ori[j,:])   )
#
#             #     self.move_to_frame(target_x, delta_t )
#             # rospy.sleep(1./sample_freq)
#     return True
