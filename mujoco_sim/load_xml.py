from mujoco_py import load_model_from_xml, MjSim, MjViewer, load_model_from_path

#  load and text a xml file.
# model = load_model_from_path("description/allegro_ycb_obj.xml")
model = load_model_from_path("description/objs.xml")
# model = load_model_from_path("description/allegro_single.xml")

sim = MjSim(model)  # MjSim represents a running simulation including its state.
viewer_ = True
if viewer_:
    viewer = MjViewer(sim)
else:
    viewer = None
    offscreen = mujoco_py.MjRenderContext(sim=sim, device_id=0, offscreen=True, opengl_backend='glfw')

while True:
    sim.step()
    viewer.render()
