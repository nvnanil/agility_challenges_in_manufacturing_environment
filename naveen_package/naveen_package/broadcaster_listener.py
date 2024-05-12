from typing import Tuple
import tf2_ros
import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose, Quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ariac_msgs.msg import (AdvancedLogicalCameraImage as AriacAdvancedLogicalCameraImage,
Part as AriacPart, PartPose as AriacPartPose)
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class BroadcasterDemo(Node):
    """
    Class to broadcast the frames
    """
    def __init__(self, node_name):
        super().__init__(node_name)

        sim_time = Parameter("use_sim_time",rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])
        
        # To get the parameter from the command line. Default value is False
        # self._listen_param = (self.declare_paramter("listen", False).get_parameter_value().bool_value)
        self._listen_param = True

        self._left_bin_parts = []
        self._right_bin_parts = []
        self._part_found = False
        #self._puple_part_found = False
        #self._blue_part_found = False
        self._part_frame={
            str(AriacPart.BLUE) : 'blue_part_frame',
            str(AriacPart.PURPLE) : 'purple_part_frame',
            str(AriacPart.RED) : 'red_part_frame',
            str(AriacPart.GREEN) : 'green_part_frame'
        }
        print(self._part_frame)
        self._part_parent_frame = None
        self._part_pose = None
        self._find_part_color = None
        self._find_part_type = None
        self._orders_cb_group = MutuallyExclusiveCallbackGroup()
        self._timer_cb_group = ReentrantCallbackGroup()
      
        # Creating Subscribers
        self._left_bin_camera_sub = self.create_subscription(
            AriacAdvancedLogicalCameraImage, 
            "ariac/sensors/left_bins_camera/image",
            self._left_bin_camera_cb,
            qos_profile_sensor_data,
        )
        
        self._right_bin_camera_sub = self.create_subscription(
            AriacAdvancedLogicalCameraImage,
            "ariac/sensors/right_bins_camera/image",
            self._right_bin_camera_cb,
            qos_profile_sensor_data,
        )
        # Timer for checking for parts
        self._callback_timer = self.create_timer(0.05, self._find_part_cb)
        
        # List of transforms to broadcast
        self._transforms = []
        
        # Creating a dynamic broadcaster
        self._tf_dynamic_broadcaster = TransformBroadcaster(self)
        
        if self._listen_param:
            # Creating a transform buffer
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self.get_logger().info("Listening")
            # Listen to the transform periodically
            self._listener_timer = self.create_timer(0.5, self.listener_callback)
        
        self.get_logger().info("Broadcasting Transforms started")
        
    
    # Assigning part and color values from the order
    
    @property
    def part_color(self):
        return self._find_part_color
    
    @property
    def part_type(self):
        return self._find_part_type
    
    @part_color.setter
    def part_color(self, value):
        self._find_part_color = value
        
    @part_type.setter
    def part_type(self, value):
            self._find_part_type = value
              
        
    def _find_part_cb(self):
        if not self._part_found:
            self.get_logger().info("Searching...")
            self.get_logger().info(f"Color: {self._find_part_color}, Type: {self._find_part_type}")
            # list_str = ', '.join([str(elem) for elem in self._left_bin_parts])
            # self.get_logger().info(f"List: {list_str}")
            for part_pose in self._left_bin_parts:
                # part_pose = AriacPartPose()
                # self.get_logger().info(f"Detected: {part_pose.part.color}")
                if (part_pose.part.color == self._find_part_color and part_pose.part.type == self._find_part_type):
                    self._part_found = True
                    self._part_pose = part_pose.pose
                    self._part_parent_frame = "left_bins_camera_frame"
                    self.generate_transform(
                        self._part_parent_frame, self._part_frame[f"{self._find_part_color}"], self._part_pose
                    )
                    self.get_logger().info("Part found in left bin")
                break
            
            for part_pose in self._right_bin_parts:
                # part_pose = AriacPartPose()
                if part_pose.part.color == self._find_part_color and part_pose.part.type == self._find_part_type:
                    self._part_found = True
                    self._part_pose = part_pose.pose
                    self._part_parent_frame = "right_bins_camera_frame"
                    self.generate_transform(
                        self._part_parent_frame, self._part_frame[f"{self._find_part_color}"], self._part_pose
                    )
                break
        else:
            # Broadcast the transforms
            self.broadcast()
            
    def broadcast(self):
        """To broadcast the generated transforms
        """
        self._tf_dynamic_broadcaster.sendTransform(self._transforms)
    
    def _left_bin_camera_cb(self, msg:AriacAdvancedLogicalCameraImage):
        self._right_bin_parts.clear()
        if len(msg.part_poses)==0:
            self.get_logger().info("No parts found")
            return
        for part_pose in msg.part_poses:
            self.get_logger().info(f"Part detected in left bins: {part_pose.part.color} color and {part_pose.part.type}")
            self._left_bin_parts.append(part_pose)
            
    def _right_bin_camera_cb(self, msg:AriacAdvancedLogicalCameraImage):
        self._right_bin_parts.clear()
        if len(msg.part_poses)==0:
            self.get_logger().info("No parts found")
            return
        for part_pose in msg.part_poses:
            self.get_logger().info(f"Part detected in right bins: {part_pose.part.color} color and {part_pose.part.type}")
            self._right_bin_parts.append(part_pose)
            
    def generate_transform(self,parent, child, pose):
        transformstamped = TransformStamped()
        
        transformstamped.header.stamp = self.get_clock().now().to_msg()
        transformstamped.header.frame_id = parent
        transformstamped.child_frame_id = child
        
        transformstamped.transform.translation.x = pose.position.x
        transformstamped.transform.translation.x = pose.position.y
        transformstamped.transform.translation.x = pose.position.z
        transformstamped.transform.rotation.x = pose.orientation.x
        transformstamped.transform.rotation.y = pose.orientation.y
        transformstamped.transform.rotation.z = pose.orientation.z
        transformstamped.transform.rotation.w = pose.orientation.w
        
        self._transforms.append(transformstamped)
        
    def listener_callback(self):
        """Callback function for the listener timer
        """
        try:
            if self._part_parent_frame is None:
                self.get_logger().warn("Parent frame is not set")
                return
            part_frame = self._part_frame[f"{self._find_part_color}"] 
            self.get_logger().info(part_frame)
            transform = self._tf_buffer.lookup_transform("world", part_frame, rclpy.time.Time())
            self.get_logger().info(f"Transformation from world frame to {part_frame}: \n " + str(transform))
            
        except TransformException as ex:
            self.get_logger().fatal(f"Could not obtain transformations between world farme and {self._part_parent_frame}")
            
            
                        
                        
            
            
        
        
        
        
        