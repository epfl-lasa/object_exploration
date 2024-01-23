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
    # lb = [-0.59471316618668479, -0.29691276729768068, -0.27401187224153672, -0.32753605719833834]
    lb = [-0.59471316618668479, 0.1, 0.1, 0.1]
    # ub = [0.57181227113054078, 1.7367399715833842, 1.8098808147084331, 1.71854352396125431]
    ub = [0.57181227113054078, 1.3, 1.3, 1.3]

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
    return torque


class Robot:
    def __init__(self, sim_handle: mujoco_py.MjSim, controller=0, exploration_list=None, viewer=None, data_log=False,
                 print_contact=False, show_contact_F=True, frame_view=None):
        self.sim = sim_handle
        self.n = sim_handle.model.nu
        self.nv = sim_handle.model.nv
        self.dt = sim_handle.model.opt.timestep
        self.name_tcp = self.sim.model.site_id2name(0)
        self.viewer = viewer
        self.log = data_log
        self.q_last = self.qh
        self.control_torque = None
        self.print_contact = print_contact

        self.frame_view = frame_view

        self.finger_tip = ['thumb_tip', 'index_tip', 'middle_tip', 'ring_tip']
        self.hand_geom_names = self.finger_tip + ["allegro_mount", "base_link_left"] + ["hand_mat" + str(i) for i in
                                                                                        range(16)]
        # the objects to be grasped
        if exploration_list is None:
            self.obj_explored = ['cylinder1', 'cylinder2']
        else:
            self.obj_explored = exploration_list
        self.exploration_obj_str_num = [len(i) for i in exploration_list]
        # print(self.exploration_obj_str_num)
        self.exploration_contact = [[], [], [],
                                    []]  # (n, 9) n is the num of contact pair. for each row  [contact position, contact force, torque] in world frame
        self.all_contacts = [[] for i in range(len(self.hand_geom_names))]
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

        # self.sim.data.set_joint_qpos('thumb_joint_0', 1)  # set the initial angle of the joint.
        # self.sim.step()  # run one step to update all position-based state

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

    def viewer_setup(self, distance=None, xyz=None, el_az=None):
        self.viewer.cam.trackbodyid = 0  # id of the body to track ()
        # self.viewer.cam.distance = self.sim.model.stat.extent * 0.05  # how much you "zoom in", model.stat.extent is the max limits of the arena
        if distance is None:
            distance = 0.675
        if xyz is None:
            xyz = [0.57, -0.31, 0.35]
        if el_az is None:
            el_az = [- 39.361422629751196, - 112.51199022662674]
        self.viewer.cam.distance = distance  # how much you "zoom in", model.stat.extent is the max limits of the arena
        self.viewer.cam.lookat[0] = xyz[0]  # x,y,z offset from the object (works if trackbodyid=-1)
        self.viewer.cam.lookat[1] = xyz[1]
        self.viewer.cam.lookat[2] = xyz[2]
        self.viewer.cam.elevation = el_az[
            0]  # camera rotation around the axis in the plane going through the frame origin (if 0 you just see a line)
        self.viewer.cam.azimuth = el_az[1]  # camera rotation around the camera's vertical axis

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
        if point_cloud.ndim == 1 and self.viewer is not None:
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

    def send_torque(self, torque, limit=True, GUI=True):
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
        if limit:
            torque = torque_limit(torque)
        self.sim.data.ctrl[:len(torque)] = torque
        self.control_torque = torque
        self.sim.step()
        if self.viewer is not None and GUI:
            self.viewer.render()
            if self.frame_view is not None:
                for i in self.frame_view:
                    self.view_frame3(i)

    def view_frame(self, pose, length=0.01):
        self.viewer.add_marker(pos=pose[:3],  # position of the arrow
                               size=np.ones(3) * length,  # size of the arrow
                               mat=rot.quat2mat(pose[3:]),  # orientation as a matrix
                               rgba=np.array([1., 0., 0., .5]),  # color of the arrow
                               type=const.GEOM_ARROW,
                               label=''
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
                                   size=np.array([0.003, 0.003, length]),  # size of the arrow
                                   mat=r,  # orientation as a matrix
                                   rgba=RGBA[i, :],  # color of the arrow
                                   type=const.GEOM_CYLINDER,  #
                                   label=''
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

    def contact_view(self, view=True):
        self.reset_contact_force()
        exploration_contact = [[], [], [], []]
        all_contacts = [[] for i in range(len(self.hand_geom_names))]
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

                # if len(c_geom_name[1]) >= self.exploration_obj_str_num[0] and c_geom_name[0] in self.finger_tip:
                #     if c_geom_name[1][:self.exploration_obj_str_num[0]] in self.obj_explored:
                #         k = self.finger_tip.index(c_geom_name[0])
                #         exploration_contact[k].append(np.concatenate([pos, F]))
                # exploration_contact.append(np.concatenate([pos, F]))
                if len(c_geom_name[1]) >= self.exploration_obj_str_num[0] and c_geom_name[0] in self.hand_geom_names:
                    if c_geom_name[1][:self.exploration_obj_str_num[0]] in self.obj_explored:
                        k = self.hand_geom_names.index(c_geom_name[0])
                        all_contacts[k].append(np.concatenate([pos, F]))

                name_intersection = [j for j in c_geom_name if j in self.obj_explored or j in self.hand_contact_force]
                # if name_intersection:
                if 1:
                    # visualize the force (3,)
                    if view:
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
            for i, data in enumerate(all_contacts):
                if data != []:
                    if len(data) == 1:
                        F_mean = data[0][0:6]
                    else:
                        all_F = np.vstack(data)

                        F_mean = np.mean(all_F[:, 0:6], axis=0)
                    self.all_contacts[i] = F_mean
                else:
                    self.all_contacts[i] = []
            self.exploration_contact = self.all_contacts[:4]

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
    def qh(self):
        """
        hand angles: thumb - index - middle - ring
        :return: (16, )
        """
        return self.sim.data.qpos[-16:]

    @property
    def dqh(self):
        """
        hand joint angular velocities
        return: (7, )
        """
        return self.sim.data.qvel[-16:]

    @property
    def ddqh(self):
        """
        hand joint angular velocities
        return: (7, )
        """
        return self.sim.data.qacc[-16:]

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
    def dxh(self):
        """
        get fingertip positions
        """
        x1 = self.sim.data.get_site_xvelp("thumb_site")
        x2 = self.sim.data.get_site_xvelp("index_site")
        x3 = self.sim.data.get_site_xvelp("middle_site")
        x4 = self.sim.data.get_site_xvelp("ring_site")

        dx = np.concatenate([x1, x2, x3, x4])
        return dx.reshape(4, 3)

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
            if jacp.shape[1] > 16:
                jacp = jacp[:, -16:]  # this is wrong, iiwa joint, free joint, hand joint
            # jacp = self.sim.data.get_site_jacp(site_name).reshape(3, 23)[:, 7 + i * 4:11 + i * 4]
            # jacr = self.sim.data.get_site_jacr(site_name).reshape(3, 23)[:, 7 + i * 4:11 + i * 4]
            J[site_name] = jacp
        return J

    ## dynamical-relevant property
    @property
    def tau_h(self):
        """
        actuator torque
        :return:
        """
        return self.sim.data.actuator_force

    @property
    def C_(self):
        """
        C(qpos,qvel), Coriolis
        :return:
        """
        return self.sim.data.qfrc_bias[-16:]


def switch_qua(q):
    """
    :param q: quaternion as [x y z w], numpy array
    :return:  quaternion as [w x y z]
    """
    return q[[3, 0, 1, 2]]


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
