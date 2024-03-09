import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data, qos_profile_services_default, qos_profile_parameters, qos_profile_parameter_events
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Marker, 'spot_state', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.init_pose_ = None 
        self.trajectory_ = []

    def listener_callback(self, msg:Odometry):
        
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id

        marker.header.stamp = msg.header.stamp
        
        marker.id = 2
        marker.type = Marker.ARROW

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 0.75
        marker.color.r = 1.0 
        marker.color.g = 1.0
        marker.color.b = 0.0

        if self.init_pose_ is None:

            self.init_pose_ = msg.pose.pose
        else:
            marker.pose = msg.pose.pose
            marker.pose.position.x -= self.init_pose_.position.x
            marker.pose.position.y -= self.init_pose_.position.y
            marker.pose.position.z = 1.0 
            self.get_logger().info(f'I heard: {marker.pose.position}')
            
            self.publisher_.publish(marker)
            marker.pose.position.z = 0.0 
            self.trajectory_.append(marker.pose.position)


    def timer_callback(self):
        
        if len(self.trajectory_) < 1:
            return
        
        marker = Marker()
        marker.header.frame_id = "odom"

        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 3
        marker.ns = "traj"
        marker.type = Marker.SPHERE_LIST

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 0.75
        marker.color.r = 1.0 
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.points = self.trajectory_
        self.publisher_.publish(marker)




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    print("spot state visualizer initialized!")
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()