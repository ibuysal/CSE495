import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import random  # For Gaussian noise
from gps_deneme_package.helper.mavlink_functions_req import *
from pymavlink import mavutil
from gps_deneme_package.helper.common import *
import time
# import os
# os.environ['MAVLINK20'] = '1'
# os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'
print("bağlanıt-yor")
connection_string = IRIS_CONNECTION_STRING_2
print("bağlandı")
master = mavutil.mavlink_connection(connection_string, source_system=255, source_component=5)
master.wait_heartbeat()
class GPSNoiseNode(Node):
    def __init__(self):
        super().__init__('gps_noise_node')
        self.home_setted = False

        self.noise_level = 0.0
        self.global_origin_counter=0
        self.gps_sub = self.create_subscription(NavSatFix, IRIS_NAVSAT_TOPIC_NAME, self.gps_callback, 1)
        self.get_logger().info("{} initialized.".format(IRIS_NAVSAT_TOPIC_NAME))

        self.noise_subscription = self.create_subscription(Float32,  IRIS_GPS_NOISE_LEVEL_TOPIC_NAME, self.noise_callback, 1)
        self.get_logger().info("GPSNoiseNode initialized.")

        self.noised_gps_pub = self.create_publisher(NavSatFix, IRIS_GPS_WITH_NOISE_TOPIC_NAME, 1)
        self.get_logger().info("Noised GPS publisher initialized: noised_gps_topic")


    def noise_callback(self, msg):
        self.noise_level = msg.data
        self.get_logger().info(f"Received GPS noise level: {self.noise_level}")

    def gps_callback(self, gps_msg):
        lat = int(round(gps_msg.latitude + random.gauss(0, self.noise_level * 1e-6), 7) * 1e7)
        lon = int(round(gps_msg.longitude + random.gauss(0, self.noise_level * 1e-6), 7) * 1e7)
        alt = gps_msg.altitude + random.gauss(0, self.noise_level * 0.1)  # Adjust altitude noise based on noise level

        noised_msg = NavSatFix()
        noised_msg.header = gps_msg.header
        noised_msg.latitude = lat/1e7
        noised_msg.longitude = lon/1e7
        noised_msg.altitude = alt
        noised_msg.position_covariance = gps_msg.position_covariance
        noised_msg.position_covariance_type = gps_msg.position_covariance_type

        if not self.home_setted:
            if self.global_origin_counter>50:
                set_default_global_origin(master, lat, lon, 584190)

                set_default_home_position(master, lat, lon, 0)
                self.home_setted = True
            self.global_origin_counter+=1
        else:

            master.mav.gps_input_send(
                0,              # time_usec
                1,              # gps_id
                4 | 8 | 16 | 32 | 128,  # ignore_flags
                0,              # time_week_ms
                0,              # time_week
                3,              # fix_type
                lat,            # Latitude (WGS84)
                lon,            # Longitude (WGS84)
                alt + 574,      # Altitude (MSL) with offset
                0,              # HDOP
                0,              # VDOP
                0,              # vn (velocity north)
                0,              # ve (velocity east)
                0,              # vd (velocity down)
                0,              # speed_accuracy
                0,              # horiz_accuracy
                0,              # vert_accuracy
                32,             # satellites_visible
                0               # yaw
            )
        #     master.mav.gps_raw_int_send(
        #       0,              #             time_usec	uint64_t	us			Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
        #       3,              # fix_type	uint8_t			GPS_FIX_TYPE	GPS fix type.
        #         lat,            # lat	int32_t	degE7			Latitude (WGS84, EGM96 ellipsoid)
        #             lon,        # lon	int32_t	degE7			Longitude (WGS84, EGM96 ellipsoid)
        #       int(alt+574)*1000 ,              # alt	int32_t	mm			Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
        #        0,             # eph	uint16_t		1E-2	invalid:UINT16_MAX	GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
        #         0,            # epv	uint16_t		1E-2	invalid:UINT16_MAX	GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
        #         0,            # vel	uint16_t	cm/s		invalid:UINT16_MAX	GPS ground speed. If unknown, set to: UINT16_MAX
        #        0,             # cog	uint16_t	cdeg		invalid:UINT16_MAX	Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        #        22,             # satellites_visible	uint8_t			invalid:UINT8_MAX	Number of satellites visible. If unknown, set to UINT8_MAX
        #         int(alt+574)*1000 ,            # alt_ellipsoid ++	int32_t	mm			Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
        #         0,            # h_acc ++	uint32_t	mm			Position uncertainty.
        #         0,            # v_acc ++	uint32_t	mm			Altitude uncertainty.
        #          0,           # vel_acc ++	uint32_t	mm/s			Speed uncertainty.
        #          0,           # hdg_acc ++	uint32_t	degE5			Heading / track uncertainty
        #          0           # yaw ++	uint16_t	cdeg		invalid:0	Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.



        # #         0,            #                time_usec	uint64_t	us		Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
        # #          3,           # fix_type	uint8_t		GPS_FIX_TYPE	GPS fix type.
        # #           lat,          # lat	int32_t	degE7		Latitude (WGS84)
        # #               lon,      # lon	int32_t	degE7		Longitude (WGS84)
        # #  int(alt+574)*1000  ,                 # alt	int32_t	mm		Altitude (MSL). Positive for up.
        # #     0,                # eph	uint16_t		invalid:UINT16_MAX	GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
        # #    0,                 # epv	uint16_t		invalid:UINT16_MAX	GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
        # #     0,                # vel	uint16_t	cm/s	invalid:UINT16_MAX	GPS ground speed. If unknown, set to: UINT16_MAX
        # #     0,                # cog	uint16_t	cdeg	invalid:UINT16_MAX	Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        # #     22,                # satellites_visible	uint8_t		invalid:UINT8_MAX	Number of satellites visible. If unknown, set to UINT8_MAX
        # #     0,                # dgps_numch	uint8_t			Number of DGPS satellites
        # #     0,                # dgps_age	uint32_t	ms		Age of DGPS info
        # #     0,                # yaw ++	uint16_t	cdeg	invalid:0	Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.
        # #   int(alt+574)*1000,                  # alt_ellipsoid ++	int32_t	mm		Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
        # #     0,                # h_acc ++	uint32_t	mm		Position uncertainty.
        # #    0,                 # v_acc ++	uint32_t	mm		Altitude uncertainty.
        # #    0,                 # vel_acc ++	uint32_t	mm/s		Speed uncertainty.
        # #    0                 # hdg_acc ++	uint32_t	degE5		Heading / track uncertainty
        #     )
        self.noised_gps_pub.publish(noised_msg)


def main(args=None):
    rclpy.init(args=args)
    gps_noise_node = GPSNoiseNode()
    rclpy.spin(gps_noise_node)

    gps_noise_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()