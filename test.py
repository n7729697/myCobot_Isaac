from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni
import carb

from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core import World
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.utils.nucleus import get_assets_root_path
import numpy as np


_command = [0.0, 0.0]

def _sub_keyboard_event(event, *args, **kwargs):
    global _command
    print(_command)
    if (event.type == carb.input.KeyboardEventType.KEY_PRESS
        or event.type == carb.input.KeyboardEventType.KEY_REPEAT):
        if event.input == carb.input.KeyboardInput.W:
            _command = [20, 0.0]
        if event.input == carb.input.KeyboardInput.S:
            _command = [-20, 0.0]
        if event.input == carb.input.KeyboardInput.A:
            _command = [0.0, np.pi / 5]
        if event.input == carb.input.KeyboardInput.D:
            _command = [0.0, -np.pi / 5]
    if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        _command = [0.0, 0.0]

# subscribe to keyboard events
appwindow = omni.appwindow.get_default_app_window()
input = carb.input.acquire_input_interface()
id=input.subscribe_to_keyboard_events(appwindow.get_keyboard(), _sub_keyboard_event)
print(id)

my_world = World(stage_units_in_meters=1)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
my_jetbot = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Jetbot",
        name="my_jetbot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jetbot_asset_path,
        position=np.array([0, 0.0, 2.0]),
    )
)
my_world.scene.add_default_ground_plane()
my_controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
my_world.reset()

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        
        my_jetbot.apply_wheel_actions(my_controller.forward(command=_command))

simulation_app.close()