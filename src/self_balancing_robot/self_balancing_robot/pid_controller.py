import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = -1.0 * (self.kp * error + self.ki * self.integral + self.kd * derivative)
        self.prev_error = error
        return output

class BalanceControllerNode(Node):
    def __init__(self):
        super().__init__('balance_controller')
        self.publisher_left = self.create_publisher(Float64, '/left_wheel/cmd_vel', 10)
        self.publisher_right = self.create_publisher(Float64, '/right_wheel/cmd_vel', 10)
        self.roll_publisher = self.create_publisher(Float64, '/measured_roll', 10)
        self.subscription = self.create_subscription(Imu, '/sensors/imu', self.imu_callback, 10)
        self.roll_setpoint_sub = self.create_subscription(Float64, '/roll_setpoint', self.setpoint_callback, 10)

        self.pid_controller = PIDController(kp=30.0, ki=70.0, kd=0.0)
        self.last_time = self.get_clock().now()
        self.roll_setpoint = 0.0  # Initialize setpoint

    def imu_callback(self, msg: Imu):
        # Extract quaternion from IMU message
        orientation_q = msg.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        
        # Convert quaternion to roll, pitch, and yaw
        roll, _, _ = euler_from_quaternion(quaternion)
        
        # Publish the measured roll
        roll_msg = Float64()
        roll_msg.data = roll
        self.roll_publisher.publish(roll_msg)
        
        # Calculate the control output using the error between setpoint and measured roll
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds

        if dt > 0:
            error = self.roll_setpoint - roll  # Error calculation
            correction = self.pid_controller.calculate(error, dt)
            self.publish_wheel_commands(correction)
            self.last_time = current_time

    def setpoint_callback(self, msg: Float64):
        # Update the roll setpoint from the topic
        self.roll_setpoint = msg.data

    def publish_wheel_commands(self, correction):
        # Apply correction to both wheels for balancing
        left_cmd = Float64()
        right_cmd = Float64()
        
        left_cmd.data = correction
        right_cmd.data = correction

        self.publisher_left.publish(left_cmd)
        self.publisher_right.publish(right_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BalanceControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
