from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core import World
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.tasks as tasks
from typing import Optional
import numpy as np

from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.importer.urdf import _urdf

import omni.kit.commands
import omni.usd


my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
# Acquire the URDF extension interface
urdf_interface = _urdf.acquire_urdf_interface()
# Set the settings in the import config
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.fix_base = True
import_config.make_default_prim = True
import_config.self_collision = False
import_config.create_physics_scene = True
import_config.import_inertia_tensor = False
import_config.default_drive_strength = 1047.19751
import_config.default_position_drive_damping = 52.35988
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.distance_scale = 1
import_config.density = 0.0
# Get the urdf file path
extension_path = get_extension_path_from_name("omni.importer.urdf")
root_path = "/home/xuezhi/mycobot_ros/mycobot_description/urdf/mycobot"
file_name = "mycobot_with_gripper_parallel.urdf"
# Finally import the robot
result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", urdf_path="{}/{}".format(root_path, file_name),
                                                import_config=import_config,)

# Inheriting from the base class Follow Target
class FollowTarget(tasks.FollowTarget):
    def __init__(
        self,
        name: str = "myCobot_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        #TODO: change this to the robot usd file.
        asset_path = "/home/xuezhi/mycobot_ros/mycobot_description/urdf/mycobot/mycobot_with_gripper_parallel/mycobot_with_gripper_parallel.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/myCobot")
        gripper = ParallelGripper(
            end_effector_prim_path="/World/myCobot/gripper_base",
            joint_prim_names=["gripper_controller", "gripper_base_to_gripper_right3"],
            joint_opened_positions=np.array([0, 0]),
            joint_closed_positions=np.array([0.628, -0.628]),
            action_deltas=np.array([-0.628, 0.628]))
        manipulator = SingleManipulator(prim_path="/World/myCobot",
                                        name="myCobot_robot",
                                        end_effector_prim_name="gripper_base",
                                        gripper=gripper)
        joints_default_positions = np.zeros(12)
        joints_default_positions[7] = 0.628
        joints_default_positions[8] = 0.628
        manipulator.set_joints_default_state(positions=joints_default_positions)
        return manipulator


#Initialize the Follow Target task with a target location for the cube to be followed by the end effector
my_task = FollowTarget(name="myCobot_follow_target", target_position=np.array([0.5, 0, 0.5]))
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("myCobot_follow_target").get_params()
target_name = task_params["target_name"]["value"]
robot_name = task_params["robot_name"]["value"]
my_cobot = my_world.scene.get_object(robot_name)
articulation_controller = my_cobot.get_articulation_controller()
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        observations = my_world.get_observations()
        print(observations)
simulation_app.close()


