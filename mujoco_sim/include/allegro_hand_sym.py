import numpy as np
import PyKDL as kdl
from urdf_parser_py.urdf import URDF

import include.kdl_parser as kdl_parser

import sympy as sy


class Robot:
    def __init__(self, start_link=None, tip_link=None, right_hand=True, use_fingers=None):
        if use_fingers is None:
            use_fingers = [1, 0, 0, 1]  # only use index and thumb fingers
        if start_link is None:
            start_link = 'base'
        if tip_link is None:
            tip_link = ['link_3.0_tip', 'link_7.0_tip', 'link_11.0_tip', 'link_15.0_tip']
        tips = []
        for i in range(len(use_fingers)):
            if use_fingers[i]:
                tips.append(tip_link[i])
        if right_hand:
            path = '/home/xiao/research/lasa/iiwa_allegro_sim/description/allegro_all/allegro_right_mount.urdf'
        else:
            # todo
            path = '/home/xiao/research/lasa/inhand_exploration/mujoco_sim/description/allegro/allegro_left_mount.urdf'
        self.start_link = start_link
        self.tip_link = tips
        self.len = len(tips)  # nums of fingers
        self.robot = URDF.from_xml_file(path)
        self.tree = kdl_parser.kdl_tree_from_urdf_model(self.robot)

        # generate chain for all fingertips
        self.chains = [self.tree.getChain(self.start_link, link) for link in self.tip_link]
        self.joint_names = [self.robot.get_chain(self.start_link, link, links=False, fixed=False) for link in
                            self.tip_link]
        print('finger number=', self.len, '  joint list:', self.joint_names)
        # print(self.tip_link)
        # forward kine
        self.fk_solver = [kdl.ChainFkSolverPos_recursive(chain) for chain in self.chains]

        # Jacobian calculation
        self.jac_calc = [kdl.ChainJntToJacSolver(chain) for chain in self.chains]

    def forward_kine(self, q, quat=True):
        """
        forward kinematics for all fingers
        :param quat: return quaternion or rotation matrix
        :param q: numpy array  (16,) or (8,)
        :return: x:  pose and jacobian
        """
        assert len(q) == 4 * self.len

        poses = []
        jacs = []
        for i in range(self.len):
            q_ = kdl_parser.joint_to_kdl_jnt_array(q[i * 4: i * 4 + 4])
            end_frame = kdl.Frame()
            self.fk_solver[i].JntToCart(q_, end_frame)
            x = np.array([end_frame.p[0], end_frame.p[1], end_frame.p[2]])
            if quat:
                qua = kdl.Rotation(end_frame.M).GetQuaternion()  # Notice that the quaternion is [x y z w]
                qua = np.array([qua[3], qua[0], qua[1], qua[2]])  # [w, x, y, z]
                pose = np.concatenate([x, qua])
                poses.append(pose)
            else:
                R = np.array([[end_frame.M[0, 0], end_frame.M[0, 1], end_frame.M[0, 2]],
                              [end_frame.M[1, 0], end_frame.M[1, 1], end_frame.M[1, 2]],
                              [end_frame.M[2, 0], end_frame.M[2, 1], end_frame.M[2, 2]]])
                T = np.concatenate([R, x.reshape(-1, 1)], axis=1)
                T = np.concatenate([T, np.array([[0, 0, 0, 1]])], axis=0)
                poses.append(T)

            jac = kdl.Jacobian(4)
            self.jac_calc[i].JntToJac(q_, jac)
            jac_array = kdl_parser.kdl_matrix_to_mat(jac)
            jacs.append(jac_array)

        return poses, jacs
