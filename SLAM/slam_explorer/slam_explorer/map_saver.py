import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String
import os

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        
        self.declare_parameter('trial_name', 'default_trial')
        self.trial_name = self.get_parameter('trial_name').value
        
        # Create Service Client
        self.cli = self.create_client(SaveMap, '/slam_toolbox/save_map')
        
        # Check service availability (don't block here forever, just check)
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('SLAM Toolbox service not available! Map saving might fail.')
            
        self.base_path = os.path.join(os.getcwd(), 'map')
        if not os.path.exists(self.base_path):
            os.makedirs(self.base_path)

        self.get_logger().info('Map Saver is ready. Press Ctrl+C to save the map and exit.')


    def save_map_on_exit(self):
        self.get_logger().info('Stopping... Attempting to save map before shutdown.')
        
        req = SaveMap.Request()
        full_path = os.path.join(self.base_path, self.trial_name)
        req.name = String(data=full_path)

        # We use a synchronous call logic here because we are in the shutdown phase
        # and async callbacks might be terminated too early.
        future = self.cli.call_async(req)
        
        # Spin specifically for this task until it completes or times out (5 seconds)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        try:
            if future.result() is not None:
                self.get_logger().info(f"SUCCESS: Map saved to {full_path}")
            else:
                self.get_logger().error("FAILURE: Service call failed/timed out.")
        except Exception as e:
            self.get_logger().error(f"ERROR: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    saver = MapSaver()
    
    try:
        rclpy.spin(saver)
    except KeyboardInterrupt:
        saver.save_map_on_exit()
    finally:
        saver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()