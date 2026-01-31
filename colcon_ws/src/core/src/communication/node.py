"""Isaac Sim ROS 2 node with parameterized robot spawning."""

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Bool
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
import numpy as np


class IsaacSimNode(Node):
    """Isaac Sim ROS 2 node with parameterized robot spawning."""

    def __init__(self, name: str = 'isaac_sim_node'):
        """Initialize Isaac Sim node.

        Args:
            name: Name of the ROS 2 node.
        """
        super().__init__(name)

        # Declare parameters
        self.declare_parameter('robot_type', 'ur5e')
        self.declare_parameter('prim_path', '/World/UR')
        self.declare_parameter('robot_name', 'ur_robot')
        self.declare_parameter('headless', False)

        # Setup ROS 2 communication
        self.status_pub = self.create_publisher(String, 'sim_status', 10)
        self.twist_pub = self.create_publisher(Float64MultiArray, 'tool0_twist', 10)
        self.joint_state_pub = self.create_publisher(Float64MultiArray, 'joint_state', 10)
        # Combined message: [q1..q6, dq1..dq6, vx, vy, vz, wx, wy, wz] = 18 values
        self.combined_state_pub = self.create_publisher(Float64MultiArray, 'robot_state_combined', 10)
        self.joint_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10
        )
        self.complete_sub = self.create_subscription(
            Bool,
            'trajectory_complete',
            self.trajectory_complete_callback,
            10
        )

        # World setup
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # Robot reference
        self.robot: Optional[object] = None
        self._target_joint_positions: Optional[np.ndarray] = None
        self._tool0_body = None
        self._publishing_twist = True

        # Log parameters
        robot_type = self.get_parameter('robot_type').value
        self.get_logger().info(f'Robot type: {robot_type}')

    def setup_robot(self):
        """Setup robot in the simulation using ur package."""
        robot_type = self.get_parameter('robot_type').value
        prim_path = self.get_parameter('prim_path').value
        robot_name = self.get_parameter('robot_name').value

        try:
            # Import ur package spawn function
            from ur.spawning import spawn_ur_robot

            self.get_logger().info(
                f'Spawning {robot_type} at {prim_path} '
                f'with name {robot_name}'
            )

            self.robot = spawn_ur_robot(
                self.world,
                robot_type=robot_type,
                prim_path=prim_path,
                name=robot_name
            )

            self.world.reset()

            # Get tool0 rigid body for velocity tracking
            self._setup_tool0_tracking(prim_path)

            # Log physics configuration
            self._log_physics_config(prim_path)

            self.get_logger().info(f'Robot {robot_type} spawned successfully')
            self.publish_status(f"Robot {robot_type} spawned and ready")

        except ImportError as e:
            self.get_logger().error(
                f'Failed to import ur package: {e}. '
                'Make sure robots/ur package is built and sourced.'
            )
            raise
        except Exception as e:
            self.get_logger().error(f'Failed to spawn robot: {e}')
            raise

    def _setup_tool0_tracking(self, prim_path: str):
        """Setup tool0 rigid body for velocity tracking.

        Args:
            prim_path: USD prim path of the robot.
        """
        try:
            from isaacsim.core.prims import RigidPrim
            from pxr import Usd

            # List available links for debugging
            stage = self.world.stage
            robot_prim = stage.GetPrimAtPath(prim_path)
            if robot_prim:
                self.get_logger().info(f"Available links under {prim_path}:")
                for child in robot_prim.GetAllChildren():
                    self.get_logger().info(f"  {child.GetPath().pathString}")

            # UR5e link paths to try (in order of preference)
            # tool0 is the end-effector frame, but might not have physics
            link_paths = [
                f"{prim_path}/tool0",
                f"{prim_path}/flange",
                f"{prim_path}/wrist_3_link",
            ]

            for link_path in link_paths:
                try:
                    # Check if prim exists
                    prim = stage.GetPrimAtPath(link_path)
                    if not prim or not prim.IsValid():
                        self.get_logger().info(f"  {link_path} - not found")
                        continue

                    self.get_logger().info(f"  {link_path} - found, type: {prim.GetTypeName()}")

                    # Try RigidPrim first (has physics velocities)
                    try:
                        self._tool0_body = RigidPrim(prim_paths_expr=link_path, name="tool0_tracking")
                        self._tool0_body.initialize()
                        self.get_logger().info(f"Tool0 tracking enabled at {link_path} (RigidPrim)")
                        return
                    except Exception as e:
                        self.get_logger().info(f"  RigidPrim failed for {link_path}: {e}")

                except Exception as e:
                    self.get_logger().info(f"  Error checking {link_path}: {e}")

            # Fallback: use wrist_3_link which should always work
            self.get_logger().warn("Falling back to wrist_3_link for tracking")
            wrist3_path = f"{prim_path}/wrist_3_link"
            self._tool0_body = RigidPrim(prim_paths_expr=wrist3_path, name="tool0_tracking")
            self._tool0_body.initialize()
            self.get_logger().info(f"Tool0 tracking enabled at {wrist3_path}")

        except Exception as e:
            self.get_logger().warn(f"Could not setup tool0 tracking: {e}")
            self._tool0_body = None

    def _log_physics_config(self, prim_path: str):
        """Log physics configuration for the robot articulation.

        Args:
            prim_path: USD prim path of the robot.
        """
        try:
            from pxr import UsdPhysics, PhysxSchema

            stage = self.world.stage
            prim = stage.GetPrimAtPath(prim_path)

            if not prim or not prim.IsValid():
                self.get_logger().warn(f"Cannot log physics config: prim not found at {prim_path}")
                return

            self.get_logger().info("=" * 50)
            self.get_logger().info("PHYSICS CONFIGURATION")
            self.get_logger().info("=" * 50)

            # Check ArticulationRootAPI
            articulation_api = UsdPhysics.ArticulationRootAPI.Get(stage, prim_path)
            if articulation_api:
                self.get_logger().info(f"ArticulationRootAPI: Applied")
            else:
                self.get_logger().info(f"ArticulationRootAPI: Not found")

            # Check PhysxArticulationAPI for self-collision
            # UR5e has PhysxArticulationAPI at {prim_path}/root_joint, not at the robot root
            paths_to_check = [prim_path, f"{prim_path}/root_joint"]
            self_collision_found = False

            for check_path in paths_to_check:
                physx_articulation = PhysxSchema.PhysxArticulationAPI.Get(stage, check_path)
                if physx_articulation:
                    self_collision_attr = physx_articulation.GetEnabledSelfCollisionsAttr()
                    if self_collision_attr:
                        self_collision_enabled = self_collision_attr.Get()
                        self.get_logger().info(
                            f"Self-Collision: {'ENABLED' if self_collision_enabled else 'DISABLED'} "
                            f"(at {check_path})"
                        )
                        self_collision_found = True
                        break

            if not self_collision_found:
                self.get_logger().info(f"Self-Collision: Not configured (PhysxArticulationAPI not found)")

            # Log joint limits
            self.get_logger().info("-" * 50)
            self.get_logger().info("JOINT LIMITS:")
            joint_names = [
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
            ]
            for joint_name in joint_names:
                joint_path = f"{prim_path}/{joint_name}"
                joint_prim = stage.GetPrimAtPath(joint_path)
                if joint_prim and joint_prim.IsValid():
                    revolute_api = UsdPhysics.RevoluteJoint.Get(stage, joint_path)
                    if revolute_api:
                        lower = revolute_api.GetLowerLimitAttr().Get()
                        upper = revolute_api.GetUpperLimitAttr().Get()
                        if lower is not None and upper is not None:
                            self.get_logger().info(f"  {joint_name}: [{lower:.2f}, {upper:.2f}] rad")
                        else:
                            self.get_logger().info(f"  {joint_name}: No limits defined")

            self.get_logger().info("=" * 50)

        except Exception as e:
            self.get_logger().warn(f"Could not log physics config: {e}")

    def trajectory_complete_callback(self, msg: Bool):
        """Handle trajectory completion signal.

        Args:
            msg: Bool message indicating trajectory completion.
        """
        if msg.data:
            self.get_logger().info("Trajectory complete - stopping twist publishing")
            self._publishing_twist = False

    def joint_command_callback(self, msg: Float64MultiArray):
        """Callback for joint commands in radians.

        Args:
            msg: Float64MultiArray containing 6 joint angles.
        """
        if self.robot is None:
            self.get_logger().warn("Robot not initialized yet")
            return

        if len(msg.data) != 6:
            self.get_logger().error(
                f"Expected 6 joint angles, got {len(msg.data)}"
            )
            return

        self.get_logger().info(f"Received joint commands: {msg.data}")
        # Store target positions for the step loop
        self._target_joint_positions = np.array(msg.data)
        self.publish_status(f"Target set to: {msg.data}")

    def publish_status(self, message: str):
        """Publish a status message to ROS 2.

        Args:
            message: Status message to publish.
        """
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)

    def step(self, render: bool = True):
        """Advance the simulation by one step.

        Args:
            render: Whether to render the simulation.
        """
        if self.robot and self._target_joint_positions is not None:
            # Apply physics-based control to reach target positions
            self.robot.apply_action(
                ArticulationAction(
                    joint_positions=self._target_joint_positions
                )
            )

        self.world.step(render=render)

        # Publish joint state and tool0 twist after physics step
        if self._publishing_twist and self.robot is not None:
            self._publish_robot_state()

    def _publish_robot_state(self):
        """Get and publish robot state from simulation.

        Publishes:
        - joint_state: [q1..q6, dq1..dq6] (12 values)
        - tool0_twist: [vx, vy, vz, wx, wy, wz] (6 values)
        - robot_state_combined: [q1..q6, dq1..dq6, vx, vy, vz, wx, wy, wz] (18 values)
        """
        try:
            # Get actual joint positions and velocities from robot
            joint_positions = self.robot.get_joint_positions()
            joint_velocities = self.robot.get_joint_velocities()

            # Get tool0 twist
            linear_vel = None
            angular_vel = None
            if self._tool0_body is not None:
                linear_vels = self._tool0_body.get_linear_velocities()
                angular_vels = self._tool0_body.get_angular_velocities()
                if linear_vels is not None and angular_vels is not None and len(linear_vels) > 0:
                    linear_vel = linear_vels[0]
                    angular_vel = angular_vels[0]

            # Publish individual topics (backward compatibility)
            if joint_positions is not None and joint_velocities is not None:
                joint_state_msg = Float64MultiArray()
                joint_state_msg.data = [float(v) for v in joint_positions] + \
                                       [float(v) for v in joint_velocities]
                self.joint_state_pub.publish(joint_state_msg)

            if linear_vel is not None and angular_vel is not None:
                twist_msg = Float64MultiArray()
                twist_msg.data = [
                    float(linear_vel[0]), float(linear_vel[1]), float(linear_vel[2]),
                    float(angular_vel[0]), float(angular_vel[1]), float(angular_vel[2])
                ]
                self.twist_pub.publish(twist_msg)

            # Publish combined message (synchronized data from same simulation step)
            if (joint_positions is not None and joint_velocities is not None and
                    linear_vel is not None and angular_vel is not None):
                combined_msg = Float64MultiArray()
                combined_msg.data = (
                    [float(v) for v in joint_positions] +
                    [float(v) for v in joint_velocities] +
                    [float(linear_vel[0]), float(linear_vel[1]), float(linear_vel[2]),
                     float(angular_vel[0]), float(angular_vel[1]), float(angular_vel[2])]
                )
                self.combined_state_pub.publish(combined_msg)

        except Exception as e:
            # Log only occasionally to avoid spam
            pass

