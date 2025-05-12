#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
import tf_transformations
import math

class CollisionBoxesPublisher(Node):
    def __init__(self):
        super().__init__('collision_boxes_publisher')
        # Create a service client for /apply_planning_scene.
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /apply_planning_scene service...")
        # Use a timer to publish the collision objects after a short delay.
        self.timer = self.create_timer(2.0, self.publish_collision_boxes)
        self.boxes_published = False

    def publish_collision_boxes(self):
        if self.boxes_published:
            return

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        collision_objects = []

        # Compute quaternion for a 90° rotation about the z-axis.
        q = tf_transformations.quaternion_from_euler(0, 0, math.pi / 2)  # [0, 0, 0.7071, 0.7071]

        # --- Box Group 1: Two boxes (dimensions: 0.83 x 0.39 x 1.85 m) ---
        # Box1:
        box1 = CollisionObject()
        box1.header.frame_id = "world"  # Adjust this if needed.
        box1.id = "box1"
        box1_shape = SolidPrimitive()
        box1_shape.type = SolidPrimitive.BOX
        box1_shape.dimensions = [0.83, 0.39, 1.85]
        box1.primitives.append(box1_shape)
        box1_pose = Pose()
        box1_pose.position.x = -0.11 - 0.1
        box1_pose.position.y = -1.7 /2
        box1_pose.position.z = (1.85 / 2.0) - ((0.838) - 0.008)   
        box1_pose.orientation.w = 1.0
        box1.primitive_poses.append(box1_pose)
        box1.operation = CollisionObject.ADD
        collision_objects.append(box1)

        # Box2 (separated by 1.7 m along x-axis):
        box2 = CollisionObject()
        box2.header.frame_id = "world"
        box2.id = "box2"
        box2_shape = SolidPrimitive()
        box2_shape.type = SolidPrimitive.BOX
        box2_shape.dimensions = [0.83, 0.39, 1.85]
        box2.primitives.append(box2_shape)
        box2_pose = Pose()
        box2_pose.position.x = -0.11 - 0.1
        box2_pose.position.y = 1.7 /2
        box2_pose.position.z = (1.85 / 2.0) - ((0.838) - 0.008)  
        box2_pose.orientation.w = 1.0
        box2.primitive_poses.append(box2_pose)
        box2.operation = CollisionObject.ADD
        collision_objects.append(box2)

        # --- Box 3: Dimensions: 1.18 x 0.59 x 0.838 m ---
        box3 = CollisionObject()
        box3.header.frame_id = "world"
        box3.id = "box3"
        box3_shape = SolidPrimitive()
        box3_shape.type = SolidPrimitive.BOX
        box3_shape.dimensions = [1.18, 0.59, 0.838]
        box3.primitives.append(box3_shape)
        box3_pose = Pose()
        box3_pose.position.x = 0.0 - 0.1
        box3_pose.position.y = 0.0  
        box3_pose.position.z = (-0.838 / 2.0) - 0.008  
        box3_pose.orientation.x = q[0]
        box3_pose.orientation.y = q[1]
        box3_pose.orientation.z = q[2]
        box3_pose.orientation.w = q[3]
        box3.primitive_poses.append(box3_pose)
        box3.operation = CollisionObject.ADD
        collision_objects.append(box3)

        # --- Box 4: Dimensions: 5.878 x 1.5 x 0.915 m ---
        box4 = CollisionObject()
        box4.header.frame_id = "world"
        box4.id = "box4"
        box4_shape = SolidPrimitive()
        box4_shape.type = SolidPrimitive.BOX
        box4_shape.dimensions = [5.878, 1.5, 0.915]
        box4.primitives.append(box4_shape)
        box4_pose = Pose()
        box4_pose.position.x = 0.955
        box4_pose.position.y = -1.88  
        box4_pose.position.z = ((-0.915 / 2.0) - 0.008) + 0.076  
        box4_pose.orientation.x = q[0]
        box4_pose.orientation.y = q[1]
        box4_pose.orientation.z = q[2]
        box4_pose.orientation.w = q[3]
        box4.primitive_poses.append(box4_pose)
        box4.operation = CollisionObject.ADD
        collision_objects.append(box4)

        # Add all boxes to the planning scene.
        planning_scene.world.collision_objects.extend(collision_objects)

        # Create and send the service request.
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info("Collision boxes applied to the planning scene.")
        else:
            self.get_logger().error("Failed to apply collision boxes.")
        self.boxes_published = True

def main(args=None):
    rclpy.init(args=args)
    node = CollisionBoxesPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
