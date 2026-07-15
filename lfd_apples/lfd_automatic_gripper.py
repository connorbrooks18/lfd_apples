import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor  # REQUIRED FOR MULTI-THREADING

from std_msgs.msg import Int16MultiArray, Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped


class GripperController(Node):

    def __init__(self):
        super().__init__('gripper_controller')

        # Create a Reentrant Callback Group to prevent service deadlocks
        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('distance_threshold', 50)   # mm
        self.declare_parameter('pressure_threshold', 600)  # hPa
        self.declare_parameter('release_timer', 5)        # sec

        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.pressure_threshold = self.get_parameter('pressure_threshold').value
        self.timer_value = self.get_parameter('release_timer').value    
        
        self.initialize_ros_topics()
        self.initialize_ros_service_clients()        
        self.initialize_ros_services() 
        self.initialize_flags()

        self.get_logger().info("GripperController node started in MANUAL-ONLY mode (Deadlock Fixed).")
        
        # Parameters
        self.apple_disposal_coord = [-0.46, 0.47, 0.22]       
        self.disposal_range = 0.05

    def initialize_ros_topics(self):
        # Subscriptions bound to the reentrant callback group
        self.distance_sub = self.create_subscription(
            Int16MultiArray,
            'microROS/sensor_data',
            self.gripper_sensors_callback,
            10,
            callback_group=self.callback_group)
        self.eef_pose_sub = self.create_subscription(
            PoseStamped,
            '/franka_robot_state_broadcaster/current_pose',
            self.eef_pose_callback,
            10,
            callback_group=self.callback_group)
        self.probing_apple_sub = self.create_subscription(
            Bool,
            'lfd/apple_probing_apple',
            self.probing_apple_callback,
            10,
            callback_group=self.callback_group)
        self.pick_done_sub = self.create_subscription(
            Bool,
            'lfd/pick_done',
            self.pick_done_callback,
            10,
            callback_group=self.callback_group)

    def initialize_ros_service_clients(self):
        # Service clients bound to the reentrant callback group
        print("HELLO\n\n\n")
        self.valve_client = self.create_client(
            SetBool, 'microROS/toggle_valve', callback_group=self.callback_group)
        self.fingers_client = self.create_client(
            SetBool, 'microROS/move_stepper', callback_group=self.callback_group)

        while not self.valve_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for valve service...')
        while not self.fingers_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for fingers service...')

    def initialize_ros_services(self):
        # Service server bound to the reentrant callback group
        self.grab_service = self.create_service(
            SetBool, 
            'gripper_grab', 
            self.gripper_grab_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info("Service server 'gripper_grab' initialized.")

    def initialize_flags(self):
        self.flag_distance = False
        self.flag_engagement = False
        self.flag_disposal = False
        self.flag_init = False
        self.cooldown = False  
        self.auto_off_timer = None
        self.flag_probing_apple = False
        self.flag_apple_picked = False
        self.flag_apple_released = False

    # ----------------------- Service Callback (Manual Override) -----------------------
    def gripper_grab_callback(self, request, response):
        if request.data:
            self.get_logger().info("External Service: GRAB Request received. Actuating hardware manually.")
            
            # 1. Turn Valve ON immediately
            req_valve = SetBool.Request()
            req_valve.data = True
            self.valve_client.call_async(req_valve)
            
            self.grab_delay_timer = self.create_timer(
                1.0, self._call_fingers_after_grab_delay, callback_group=self.callback_group)
            response.success = True
            response.message = "Gripper explicitly instructed to GRAB."
        else:
            self.get_logger().info("External Service: RELEASE Request received. Resetting hardware manually.")
            self.fingers_and_valve_reset()
            response.success = True
            response.message = "Gripper explicitly instructed to RELEASE."

        return response

    # ----------------------- Helper to safely destroy timers -----------------------
    def destroy_timer_safe(self, attr_name: str):
        t = getattr(self, attr_name, None)
        if t is not None:
            try:
                t.cancel()
                self.destroy_timer(t)
            except Exception as e:
                self.get_logger().warn(f"Error destroying timer {attr_name}: {e}")
            setattr(self, attr_name, None)

    def _call_fingers_after_grab_delay(self):
        # Always destroy the one-shot timer immediately so it doesn't loop
        self.destroy_timer_safe("grab_delay_timer")
        
        self.get_logger().info("Delay complete. Extending fingers manually.")
        
        # 3. Deploy Fingers
        req_fingers = SetBool.Request()
        req_fingers.data = True
        future_fingers = self.fingers_client.call_async(req_fingers)
        future_fingers.add_done_callback(self._after_manual_fingers_extended)

    def _after_manual_fingers_extended(self, future_fingers: Future):
        try:
            response = future_fingers.result()
            if response.success:
                self.get_logger().info("Successful Manual Fingers Out.")
            else:
                self.get_logger().warn("Failed Manual Fingers Out.")
        except Exception as e:
            self.get_logger().error(f"Manual Fingers Out service call failed: {e}")

    # ----------------------- REMOVED AUTOMATIC LOOPS -----------------------
    def gripper_sensors_callback(self, msg: Int16MultiArray):
        pass

    def auto_off_callback(self):
        pass

    def eef_pose_callback(self, msg: PoseStamped):
        pass

    def probing_apple_callback(self, msg: Bool):
        self.flag_probing_apple = msg.data

    def pick_done_callback(self, msg: Bool):
        self.flag_apple_picked = msg.data

    # ----------------------- Reset Sequence -----------------------
    def fingers_and_valve_reset(self):
        self.get_logger().info("Resetting fingers and valve to initial state.")
        req = SetBool.Request()
        req.data = False
        future_fingers = self.fingers_client.call_async(req)
        future_fingers.add_done_callback(self._after_fingers_call_reset)

    def _after_fingers_call_reset(self, future_fingers: Future):
        try:
            response = future_fingers.result()
            if response.success:
                self.get_logger().info("Successful Fingers In.")
            else:
                self.get_logger().warn("Failed Fingers In.")
        except Exception as e:
            self.get_logger().error(f"Fingers In service call failed: {e}")
        
        # Explicitly assign timer to the callback group so it can execute concurrently
        self.delay_timer = self.create_timer(
            0.5, self._call_valve_after_reset, callback_group=self.callback_group)

    def _call_valve_after_reset(self):
        self.destroy_timer_safe("delay_timer")
        req_valve = SetBool.Request()
        req_valve.data = False
        future_valve = self.valve_client.call_async(req_valve)
        future_valve.add_done_callback(self._after_valve_call_reset)

    def _after_valve_call_reset(self, future_valve: Future):
        try:
            response = future_valve.result()
            if response.success:
                self.get_logger().info("Successful Valve Closed.")
            else:
                self.get_logger().warn("Failed Valve Closed.")
        except Exception as e:
            self.get_logger().error(f"Valve service call failed: {e}")
            
        self.delay_reset_timer = self.create_timer(
            0.5, self._after_reset_complete, callback_group=self.callback_group)

    def _after_reset_complete(self):
        self.destroy_timer_safe("delay_reset_timer")
        self.flag_distance = False
        self.flag_engagement = False        
        self.get_logger().info("Reset complete. System idle waiting for next client command.")


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    
    # Switch from single-threaded to MultiThreadedExecutor to break the callback block
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
