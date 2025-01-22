from pymavlink import mavutil
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from gps_deneme_package.helper.common import *
import time
class GimbalController(Node):
    def __init__(self):
        super().__init__('gimbal_controller')

        self.subscription = self.create_subscription(
            Imu,
            '/imu', 
            self.imu_callback,
            10
        )
        self.subscription 

        self.mavlink_connection = mavutil.mavlink_connection(IRIS_CONNECTION_STRING_5)
        self.mavlink_connection.wait_heartbeat()
        self.get_logger().info('MAVLink bağlantısı sağlandı.')

        self.pitch_min_deg = -90 
        self.pitch_max_deg = 90  
        self.pitch_min_pwm = 1100 
        self.pitch_max_pwm = 1900 

        self.roll_min_deg = -90  
        self.roll_max_deg = 90   
        self.roll_min_pwm = 1100 
        self.roll_max_pwm = 1900  
        self.last_servo_time = time.time()
        self.servo_update_interval = 0.03
    def imu_callback(self, msg):
        current_time = time.time()

        if current_time - self.last_servo_time >= self.servo_update_interval:
            self.last_servo_time = current_time
            orientation = msg.orientation
            roll, pitch, _ = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

            roll_pwm = self.map_angle_to_pwm(roll, self.roll_min_deg, self.roll_max_deg, self.roll_min_pwm, self.roll_max_pwm)
            pitch_pwm = self.map_angle_to_pwm(pitch, self.pitch_min_deg, self.pitch_max_deg, self.pitch_min_pwm, self.pitch_max_pwm)
            
            self.send_servo_command(channel=5, pwm=roll_pwm)  
            self.send_servo_command(channel=6, pwm=pitch_pwm) 

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """
        Quaternion'dan Euler açılarına dönüşüm.
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    @staticmethod
    def map_angle_to_pwm(angle, min_angle, max_angle, min_pwm, max_pwm):
        """
        Bir açı değerini verilen PWM aralığına dönüştür.
        """
        angle = max(min(angle, max_angle), min_angle)  # Açıyı sınırla
        pwm = min_pwm + (max_pwm - min_pwm) * (angle - min_angle) / (max_angle - min_angle)
        return int(pwm)

    def send_servo_command(self, channel, pwm):
        """
        Servo için PWM komutunu gönder.
        :param channel: Servo kanalı (örneğin, 1: Roll, 2: Pitch)
        :param pwm: Servo için PWM değeri
        """
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 
            0,                                    
            channel,                              
            pwm,                                  
            0, 0, 0, 0, 0                        
        )
        # self.get_logger().info(f"Servo komutu gönderildi: Kanal={channel}, PWM={pwm}")


def main(args=None):
    rclpy.init(args=args)
    gimbal_controller = GimbalController()
    try:
        rclpy.spin(gimbal_controller)
    except KeyboardInterrupt:
        gimbal_controller.get_logger().info('Kapatılıyor...')
    finally:
        gimbal_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
