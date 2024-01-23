import time
import rospy
from std_msgs.msg import Float64MultiArray

from mujoco_py import load_model_from_xml, MjSim, MjViewer, load_model_from_path
import mujoco_py
import numpy as np
# import matplotlib.pyplot as plt
# import os
# import sys
import include.controllers_utils_no_iiwa as controllers_utils
import include.rotations as rot

import include.allegro_hand_sym as allegro_hand_ik

from PIL import Image

# obj_name = 'apc_1'
# obj_name = 'apc_2x2'
# obj_name = 'apc_red_bowl'
obj_name = 'bunny'              #  for this mesh, a 90 degree rotation along x axis, already set in .xml
# obj_name = 'apc_redcup'
# obj_name = 'finger_tip'
# obj_name = 'apc_2x2'

# load records
suffix = 20015
replay_best = True
best_data_num = [186, 601, 506, 822]
# best_data_num = [193, 601, 506, 822]
obj_names = ['finger_tip', 'apc_2x2', 'apc_red_bowl', 'bunny']

if replay_best:
    assert obj_name in obj_names
    tmp = obj_names.index(obj_name)
    suffix = best_data_num[tmp]

data_path = 'records/' + obj_name + '_' + str(suffix) + '.npy'
print('data load path:', data_path)

palm_impedance = False
show_contact_force = False
contact_point_size = 0.004

if palm_impedance:
    xml_path = 'description/inhand_exploration_' + obj_name + '2.xml'
else:
    xml_path = 'description/inhand_exploration_' + obj_name + '.xml'

print(xml_path)
model = load_model_from_path(xml_path)

sim = MjSim(model)  # MjSim represents a running simulation including its state.
viewer_ = True
if viewer_:
    # if report an error, `_hide_overlay=False` can be deleted.
    viewer = MjViewer(sim, _hide_overlay=True)
else:
    viewer = None
    # No viewer
    # viewer = mujoco_py.MjRenderContext(sim=sim, device_id=0, offscreen=True, opengl_backend='glfw')
    viewer = mujoco_py.MjRenderContextOffscreen(sim)

r = controllers_utils.Robot(sim, 0, viewer=viewer, data_log=False, print_contact=False, exploration_list=[obj_name],
                            frame_view=['allegro_base', 'allegro_base_mocap'])


if obj_name == 'bunny':
    r.viewer_setup(distance=0.5, xyz=[-0.00249776, 0.00362528, 0.45], el_az=[-28, -75])
elif obj_name == 'apc_red_bowl':
    r.viewer_setup(distance=0.5, xyz=[-0.00249776, 0.00362528, 0.05], el_az=[-40, -75])
elif obj_name == 'finger_tip':
    r.viewer_setup(distance=0.734, xyz=[-0.00249776, 0.00362528, 0.15], el_az=[-28, -75])
else:
    r.viewer_setup(distance=0.734, xyz=[-0.00249776, 0.00362528, 0.25], el_az=[-28, -75])

# img.show()

data_records = np.load(data_path)
down_sampling = 1
data_records = data_records[::down_sampling, :]
timestamp = None
steps = None
if data_records.shape[1] == 1 + 7 + 16 + 6 * 22:
    timestamp = data_records[:, 0]
    data_records = data_records[:, 1:]
elif data_records.shape[1] == 2 + 7 + 16 + 6 * 22:
    timestamp = data_records[:, 0]
    steps = data_records[:, 1]
    data_records = data_records[:, 2:]
else:
    raise NotImplementedError

q_rest = np.zeros(16)  # rest joints, to be set in null space
q_rest[0] = 0.5  # thumb
q_rest[[1, 2, 3]] = 0.4
q_rest[[5, 6, 7, 9, 10, 11, 13, 14, 15]] = 0.3
r.set_left_hand_joints(q_rest)

contact_color = [np.array([1, 0, 0, 1]), np.array([0, 1, 0, 1]), np.array([0, 0, 1, 1]), np.array([1, 1, 0, 1])]

contact_points = [[], [], [], []]

first_pose = True
mocap = True
i = 0
screenshot_num = 0
while 1:
    p_wrist2world = data_records[i, :7]
    qh = data_records[i, 7:23]
    contact = data_records[i, 23:].reshape(22, 6)

    if mocap:
        r.sim.data.mocap_pos[:] = p_wrist2world[:3]  # apply the pose to the base link of allegro
        r.sim.data.mocap_quat[:] = p_wrist2world[3:]

    r.set_left_hand_joints(qh)
    # r.contact_view()
    # r.view_frame(p_wrist2world)
    if show_contact_force:
        for j in range(22):
            if np.linalg.norm(contact[j, :]) > 1e-5:
                r.view_vector(contact[j, :3], contact[j, 3:])  # input contact position and contact force vector

    threshold = 0.03
    for j in range(4):
        if np.linalg.norm(contact[j, :]) > 1e-5:
            if len(contact_points[j]) != 0:
                if np.linalg.norm(contact[j, :]) > 1e-5 and np.linalg.norm(
                        contact_points[j][-1, :3] - contact[j, :3]) > threshold:
                    contact_points[j] = np.vstack([contact_points[j], contact[j, :3].reshape(1, -1)])
            else:
                contact_points[j] = contact[j, :3].reshape(1, -1)

        if len(contact_points[j]) != 0:
            r.view_obj_mat(contact_points[j], size=contact_point_size, color=contact_color[j], nums=0)

    # store the last contact
    # contact_points = [contact[i, :] for i in range(4)]  # fingertip contacts, if zeros means no contacts

    if viewer_:
        r.viewer.render()


    sample_nums = 200
    tmp = int(data_records.shape[0] / sample_nums)
    if i % tmp == 0 or i == data_records.shape[0] - 1:
        screenshot_num += 1
        nums = sum([len(tmp) for tmp in contact_points])
        print(i / data_records.shape[0], '    geom nums', nums, '   viewer setting:',
              [r.viewer.cam.distance, r.viewer.cam.lookat[:], r.viewer.cam.elevation, r.viewer.cam.azimuth])
        # https://github.com/openai/mujoco-py/issues/201
        # https://github.com/openai/mujoco-py/issues/423
        # rgb_array = r.sim.render(width=2560,
        #                          height=1440)  # if need to save figures on the fly, we have to use offscreen render.

        if not viewer_:
            r.viewer.render(2550, 1440)
            rgb_array = np.asarray(r.viewer.read_pixels(2550, 1440, depth=False)[::-1, :, :], dtype=np.uint8)
            # del viewer._markers[:]

            ## repair it to avoid not being purely white [255,255,255]
            rgb_array_index = rgb_array == 252
            rgb_array[np.all(rgb_array_index, axis=2), :] = 255
            # rgb_array = np.rot90(rgb_array, 2)
            img = Image.fromarray(rgb_array, 'RGB')
            img.save('records/' + obj_name + '/' + 'GP_' + str(suffix) + '_' + str(screenshot_num) + '.png')
    r.viewer._markers = []
    i += 1
    if i == data_records.shape[0]:
        break

print('Finished!')
if timestamp is not None:
    print('Time cost', timestamp[-1])

# while 1:
#     r.view_obj_mat(contact_points[j], size=contact_point_size, color=contact_color[j], nums=0)
#     r.viewer.render(2550, 1440)
#     r.viewer._markers = []
#     r.sim.step()
#     time.sleep(0.001)
