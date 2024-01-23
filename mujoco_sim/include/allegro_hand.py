import numpy as np
import PyKDL as kdl
from urdf_parser_py.urdf import URDF

import include.kdl_parser as kdl_parser

class Robot:
    def __init__(self, start_link=None, tip_link=None, right_hand=True, use_fingers=None, path_prefix='',
                 all_link_fk=False, meshes=False):
        if use_fingers is None:
            use_fingers = [1, 0, 0, 1]  # only use index and thumb fingers
        if start_link is None:
            start_link = 'base'
        if tip_link is None:
            if right_hand:
                tip_link = ['link_3.0_tip', 'link_7.0_tip', 'link_11.0_tip', 'link_15.0_tip']
            else:
                tip_link = ['link_3_tip', 'link_7_tip', 'link_11_tip', 'link_15_tip']  # todo modify urdf file
        tips = []
        self.use_fingers = use_fingers
        for i in range(len(use_fingers)):
            if use_fingers[i]:
                tips.append(tip_link[i])
        if right_hand:
            path = path_prefix + 'description/allegro_all/allegro_right_mount.urdf'
            if meshes:
                path = path_prefix + 'description/allegro_all/allegro_right_mount_meshes.urdf'
        else:
            # todo
            path = path_prefix + 'description/allegro/allegro_left_mount.urdf'
            if meshes:
                path = path_prefix + 'description/allegro_all/allegro_left_mount_meshes.urdf'
            
        self.start_link = start_link
        self.tip_link = tips
        self.len = len(tips)  # nums of fingers
        self.robot = URDF.from_xml_file(path)
        self.tree = kdl_parser.kdl_tree_from_urdf_model(self.robot)

        # generate chain for all fingertips
        self.chains = [self.tree.getChain(self.start_link, link) for link in self.tip_link]
        self.joint_names = [self.robot.get_chain(self.start_link, link, links=False, fixed=False) for link in
                            self.tip_link]
        self.link_names = [self.robot.get_chain(self.start_link, link, joints=False, links=True, fixed=False) for link
                           in self.tip_link]
        print(use_fingers)
        print('finger number=', self.len, '  joint list:', self.joint_names)
        # print(self.tip_link)
        if all_link_fk:
            all_links = []
            for i in self.link_names:
                for j in i:
                    if j[-2:] == '00':
                        j = j[:-2]
                    if j not in all_links:
                        all_links.append(j)
            if all_links[0] == 'base':
                all_links = all_links[1:]
            self.all_links = all_links
            self.all_chains = [self.tree.getChain(self.start_link, link) for link in self.all_links]
            self.fk_solver_all = [kdl.ChainFkSolverPos_recursive(chain) for chain in self.all_chains]

        # forward kine
        self.fk_solver = [kdl.ChainFkSolverPos_recursive(chain) for chain in self.chains]

        # Jacobian calculation
        self.jac_calc = [kdl.ChainJntToJacSolver(chain) for chain in self.chains]

        # joint limit from hand_dexterity of Kunpeng repo
        lb = [-0.59471316618668479, -0.29691276729768068, -0.27401187224153672, -0.32753605719833834] * 3 + [
            0.3635738998060688, -0.20504289759570773, -0.28972295140796106, -0.26220637207693537]
        ub = [0.57181227113054078, 1.7367399715833842, 1.8098808147084331, 1.71854352396125431] * 3 + [
            1.4968131524486665, 1.2630997544532125, 1.7440185506322363, 1.8199110516903878]
        self.joint_lb = np.array(lb)
        self.joint_ub = np.array(ub)

    def forward_kine(self, q, quat=True, return_jac=True):
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
            if return_jac:
                jac = kdl.Jacobian(4)
                self.jac_calc[i].JntToJac(q_, jac)
                jac_array = kdl_parser.kdl_matrix_to_mat(jac)
                jacs.append(jac_array)

        if return_jac:
            return poses, jacs
        else:
            return poses

    def get_vel(self, q, dq):

        _, jacs = self.forward_kine(q)
        vel = []
        for i in range(self.len):
            v = jacs[i] @ dq[i * 4: i * 4 + 4].reshape(-1, 1)
            vel.append(v)
        return vel

    def get_all_links(self, q, link_index=None):

        # pass
        poses = []
        if link_index is None:
            nums = list(range(len(self.all_links)))
        else:
            nums = link_index

        for i in nums:
            end_frame = kdl.Frame()
            if i < 2:  # allegro_mount and palm link
                q_ = kdl_parser.joint_to_kdl_jnt_array([])
            else:  # index finger
                num_finger = (i - 2) // 5
                num_link = (i - 2) % 5 + 1
                if num_link == 5:
                    num_link = 4
                a = num_finger * 4
                b = a + num_link
                q_ = kdl_parser.joint_to_kdl_jnt_array(q[a:b])
            self.fk_solver_all[i].JntToCart(q_, end_frame)
            x = np.array([end_frame.p[0], end_frame.p[1], end_frame.p[2]])
            qua = kdl.Rotation(end_frame.M).GetQuaternion()  # Notice that the quaternion is [x y z w]
            qua = np.array([qua[3], qua[0], qua[1], qua[2]])  # [w, x, y, z]
            poses.append(np.concatenate([x, qua]))
        return poses

    def generate_rand_joints(self, num, reset_seed=True):
        # q_rand = []
        # for i in range(len(self.joint_lb)):
        #     q_tmp = np.random.uniform(self.joint_lb[i], self.joint_ub[i], num)
        #     q_rand.append(q_tmp)
        #
        # q_rand = np.asarray(q_rand).T
        if reset_seed:
            np.random.seed() # reset the seed for multi processing, otherwise they will have the same results form random
        samples = np.random.uniform(self.joint_lb, self.joint_ub, size=(num, 16))
        return samples  # shape (num, 16)
