
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
connection_string = IRIS_1_CONNECTION_STRING_1
print("bağlandı")
master = mavutil.mavlink_connection(connection_string, source_system=255, source_component=5)
master.wait_heartbeat()

class GPSNoiseNode(Node):
    def __init__(self):
        super().__init__('gps_noise_node')
        self.home_setted = False

        self.noise_level = 0.0
        self.global_origin_counter=0
        self.gps_sub = self.create_subscription(NavSatFix, IRIS_1_NAVSAT_TOPIC_NAME, self.gps_callback, 1)
        self.get_logger().info("{} initialized.".format(IRIS_1_NAVSAT_TOPIC_NAME))

        self.noise_subscription = self.create_subscription(Float32,  IRIS_1_GPS_NOISE_LEVEL_TOPIC_NAME, self.noise_callback, 1)
        self.get_logger().info("GPSNoiseNode initialized.")

        self.noised_gps_pub = self.create_publisher(NavSatFix, IRIS_1_GPS_WITH_NOISE_TOPIC_NAME, 1)
        self.get_logger().info("Noised GPS publisher initialized: noised_gps_topic")


    def noise_callback(self, msg):
        self.noise_level = msg.data
        self.get_logger().info(f"Received GPS noise level: {self.noise_level}")

    def gps_callback(self, gps_msg):
        # Apply Gaussian noise to latitude, longitude, and altitude using the determined noise level
        lat = int(round(gps_msg.latitude + random.gauss(0, self.noise_level * 1e-6), 7) * 1e7)
        lon = int(round(gps_msg.longitude + random.gauss(0, self.noise_level * 1e-6), 7) * 1e7)
        alt = gps_msg.altitude + random.gauss(0, self.noise_level * 0.1)  # Adjust altitude noise based on noise level

        # print(f"Latitude with noise: {lat/1e7}")
        # print(f"Longitude with noise: {lon/1e7}")
        # print(f"Altitude with noise: {alt}")
        noised_msg = NavSatFix()
        noised_msg.header = gps_msg.header
        noised_msg.latitude = lat/1e7
        noised_msg.longitude = lon/1e7
        noised_msg.altitude = alt
        noised_msg.position_covariance = gps_msg.position_covariance
        noised_msg.position_covariance_type = gps_msg.position_covariance_type

        # Set the home position and global origin once
        if not self.home_setted:
            if self.global_origin_counter>50:
                set_default_global_origin(master, lat, lon, 584190)

                set_default_home_position(master, lat, lon, 0)
                self.home_setted = True
            self.global_origin_counter+=1
        else:

            # Send GPS input message with noise-added values
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
                22,             # satellites_visible
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
# # scripts/script_module.py


# # scripts/script_module.py
# # scripts/script_module.py
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from geometry_msgs.msg import PointStamped
# from std_msgs.msg import Float32
# import random  # For Gaussian noise
# from pymavlink import mavutil
# from . import mavlink_functions_req as mfr

# from gps_deneme_package.helper.common import *
# from pyproj import Transformer  # For LLH to ECEF conversion

# print("Connecting...")
# connection_string = IRIS_1_CONNECTION_STRING_1
# print("Connected")
# master = mavutil.mavlink_connection(connection_string, source_system=255, source_component=5)
# master.wait_heartbeat()

# class GPSNoiseNode(Node):
#     def __init__(self):
#         super().__init__('gps_noise_node')
#         self.home_setted = False

#         # Default noise level if no gps_noise messages are received
#         self.noise_level = 1.0

#         # Transformer for LLH to ECEF conversion
#         self.llh_to_ecef = Transformer.from_crs("+proj=latlong +datum=WGS84", "+proj=geocent +datum=WGS84")
        
#         # Subscriber for NavSatFix messages
#         self.gps_sub = self.create_subscription(NavSatFix, IRIS_1_NAVSAT_TOPIC_NAME, self.gps_callback, 10)
#         self.get_logger().info("{} initialized.".format(IRIS_1_NAVSAT_TOPIC_NAME))

#         # Subscriber for gps_noise messages, if available
#         self.noise_subscription = self.create_subscription(Float32,  IRIS_1_GPS_NOISE_TOPIC_NAME, self.noise_callback, 10)
        
#         # Publisher for noised GPS data in Cartesian coordinates
#         self.cartesian_pub = self.create_publisher(PointStamped, IRIS_1_GPS_NOISE_CARTESIAN_TOPIC_NAME, 10)

#         self.get_logger().info("GPSNoiseNode initialized.")

#     def noise_callback(self, msg):
#         # Update noise level based on the received gps_noise message
#         self.noise_level = msg.data
#         self.get_logger().info(f"Received GPS noise level: {self.noise_level}")

#     def gps_callback(self, gps_msg):
#         # Apply Gaussian noise to latitude, longitude, and altitude using the determined noise level
#         lat_noised = gps_msg.latitude + random.gauss(0, self.noise_level * 1e-6)
#         lon_noised = gps_msg.longitude + random.gauss(0, self.noise_level * 1e-6)
#         alt_noised = gps_msg.altitude + random.gauss(0, self.noise_level * 0.1)  # Adjust altitude noise based on noise level

#         print(f"Latitude with noise: {lat_noised}")
#         print(f"Longitude with noise: {lon_noised}")
#         print(f"Altitude with noise: {alt_noised}")

#         # Set the home position and global origin once
#         if not self.home_setted:
#             set_default_global_origin(master, lat_noised, lon_noised, 0)
#             set_default_home_position(master, lat_noised, lon_noised, 0)
#             self.home_setted = True

#         # Convert LLH (Lat, Lon, Alt) to ECEF (X, Y, Z)
#         x, y, z = self.llh_to_ecef.transform(lon_noised, lat_noised, alt_noised)

#         # Publish the Cartesian coordinates as a PointStamped message
#         cartesian_msg = PointStamped()
#         cartesian_msg.header.stamp = self.get_clock().now().to_msg()
#         cartesian_msg.header.frame_id = "map"
#         cartesian_msg.point.x = x
#         cartesian_msg.point.y = y
#         cartesian_msg.point.z = z

#         self.cartesian_pub.publish(cartesian_msg)
#         self.get_logger().info(f"Published Cartesian Coordinates: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

#         # Send GPS input message with noise-added values
#         master.mav.gps_input_send(
#             0,              # time_usec
#             1,              # gps_id
#             4 | 8 | 16 | 32 | 128,  # ignore_flags
#             0,              # time_week_ms
#             0,              # time_week
#             3,              # fix_type
#             int(round(lat_noised * 1e7)),  # Latitude (WGS84)
#             int(round(lon_noised * 1e7)),  # Longitude (WGS84)
#             alt_noised + 574,  # Altitude (MSL) with offset
#             0,              # HDOP
#             0,              # VDOP
#             0,              # vn (velocity north)
#             0,              # ve (velocity east)
#             0,              # vd (velocity down)
#             0,              # speed_accuracy
#             0,              # horiz_accuracy
#             0,              # vert_accuracy
#             22,             # satellites_visible
#             0               # yaw
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     gps_noise_node = GPSNoiseNode()
#     rclpy.spin(gps_noise_node)

#     gps_noise_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# import random  # For Gaussian noise
# from . import mavlink_functions_req as mfr
# from pymavlink import mavutil
# from gps_deneme_package.helper.common import *
# import time

# print("bağlanıt-yor")
# connection_string = IRIS_1_CONNECTION_STRING_1
# print("bağlandı")
# master = mavutil.mavlink_connection(connection_string, source_system=255, source_component=5)
# master.wait_heartbeat()

# class SensorSync(Node):
#     def __init__(self):
#         super().__init__('sensor_sync')
#         self.home_setted = False

#         # Subscriber for NavSatFix only
#         self.gps_sub = self.create_subscription(NavSatFix, IRIS_1_NAVSAT_TOPIC_NAME, self.gps_callback, 10)

#     def gps_callback(self, gps_msg):
#         # Adding Gaussian noise to latitude, longitude, and altitude
#         noise_level = 0.00001  # Adjust this value to control noise level
#         lat = int(round(gps_msg.latitude + random.gauss(0, noise_level), 7) * 1e7)
#         lon = int(round(gps_msg.longitude + random.gauss(0, noise_level), 7) * 1e7)
#         alt = gps_msg.altitude + random.gauss(0, noise_level * 1e2)  # Adjust altitude noise

#         print(f"Latitude with noise: {lat}")
#         print(f"Longitude with noise: {lon}")
#         print(f"Altitude with noise: {alt}")

#         # Set the home position and global origin once
#         if not self.home_setted:
#             set_default_global_origin(master, lat, lon, 0)
#             set_default_home_position(master, lat, lon, 0)
#             self.home_setted = True
#         else:
#             # Send GPS input message with noise-added values
#             master.mav.gps_input_send(
#                 0,              # time_usec
#                 1,              # gps_id
#                 4 | 8 | 16 | 32 | 128,  # ignore_flags
#                 0,              # time_week_ms
#                 0,              # time_week
#                 3,              # fix_type
#                 lat,            # Latitude (WGS84)
#                 lon,            # Longitude (WGS84)
#                 alt + 574,      # Altitude (MSL) with offset
#                 0,              # HDOP
#                 0,              # VDOP
#                 0,              # vn (velocity north)
#                 0,              # ve (velocity east)
#                 0,              # vd (velocity down)
#                 0,              # speed_accuracy
#                 0,              # horiz_accuracy
#                 0,              # vert_accuracy
#                 22,             # satellites_visible
#                 0               # yaw
#             )
#             print("GPS input message with noise sent.")

# def main(args=None):
#     rclpy.init(args=args)
#     sensor_sync = SensorSync()
#     rclpy.spin(sensor_sync)

#     sensor_sync.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix,FluidPressure
# from message_filters import ApproximateTimeSynchronizer, Subscriber
# from . import mavlink_functions_req as mfr
# from pymavlink import mavutil
# from gps_deneme_package.helper.common import *
# import time
# print("bağlanıt-yor")
# connection_string = IRIS_1_CONNECTION_STRING_1
# print("bağlandı")
# master = mavutil.mavlink_connection(connection_string,source_system=255,source_component=5)
# master.wait_heartbeat()

# # print(master.target_system,master.target_component)
# # 1 0

# # set_parameter(master,"GPS1_TYPE",0)
# # # set_parameter(master,"GPS_AUTO_SWITCH",0)

# # # set_parameter(master,"GPS_PRIMARY",1)

# # set_parameter(master,"GPS2_TYPE",14)


# # set_parameter(master,"SR0_EXT_STAT",50)
# # set_parameter(master,"EK3_SRC1_VELXY",0)

# # set_parameter(master,"GPS_AUTO_CONFIG",0)
# class SensorSync(Node):
#     def __init__(self):
#         super().__init__('sensor_sync')
#         self.home_setted=False

#         # Subscribers with message filters
#         self.gps_sub = Subscriber(self, NavSatFix, IRIS_1_NAVSAT_TOPIC_NAME)
#         self.pressure_sub = Subscriber(self, FluidPressure, IRIS_1_AIR_PRES_TOPIC_NAME)
        
#         # ApproximateTimeSynchronizer to synchronize the two topics
#         self.ts = ApproximateTimeSynchronizer(
#             [self.gps_sub, self.pressure_sub],
#             queue_size=10,
#             slop=0.1  # Time tolerance to match messages (adjust based on data rate)
#         )
#         self.ts.registerCallback(self.sync_callback)

#     def sync_callback(self, gps_msg, pressure_msg):
#         # This callback will be called when both messages are received within the tolerance
#         # self.get_logger().info('Synchronized messages:')
#         # self.get_logger().info(f'GPS - Latitude: {gps_msg.latitude}, Longitude: {gps_msg.longitude}')
#         # self.get_logger().info(f'Pressure - Fluid Pressure: {pressure_msg.fluid_pressure}')
#         lat=int(round(gps_msg.latitude,7)*1e7)
#         lon=int(round(gps_msg.longitude,7)*1e7)
#         alt=(gps_msg.altitude)
#         print(lat)
#         print( lon)
#         print(alt)

#         if not self.home_setted:
#             set_default_global_origin(master,lat,lon,0)
#             set_default_home_position(master,lat,lon,0)
#             self.home_setted=True
#         else:
# #             Field Name	Type	Units	Values	Description
# # time_usec	uint64_t	us		Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
# # gps_id	uint8_t			ID of the GPS for multiple GPS inputs
# # Messages with same value are from the same source (instance).
# # ignore_flags	uint16_t		GPS_INPUT_IGNORE_FLAGS	Bitmap indicating which GPS input flags fields to ignore. All other fields must be provided.
# # time_week_ms	uint32_t	ms		GPS time (from start of GPS week)
# # time_week	uint16_t			GPS week number
# # fix_type	uint8_t			0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
# # lat	int32_t	degE7		Latitude (WGS84)
# # lon	int32_t	degE7		Longitude (WGS84)
# # alt	float	m		Altitude (MSL). Positive for up.
# # hdop	float		invalid:UINT16_MAX	GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
# # vdop	float		invalid:UINT16_MAX	GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
# # vn	float	m/s		GPS velocity in north direction in earth-fixed NED frame
# # ve	float	m/s		GPS velocity in east direction in earth-fixed NED frame
# # vd	float	m/s		GPS velocity in down direction in earth-fixed NED frame
# # speed_accuracy	float	m/s		GPS speed accuracy
# # horiz_accuracy	float	m		GPS horizontal accuracy
# # vert_accuracy	float	m		GPS vertical accuracy
# # satellites_visible	uint8_t			Number of satellites visible.
# # yaw ++	uint16_t	cdeg		Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north
#             master.mav.gps_input_send(
#                    0,
#                     1,
#                     4|8|16|32|128,
#                     0,
#                     0,
#                     3,
#                     lat,
#                     lon,#lon
#                     alt+574,#alt
#                     0,
#                     0,
#                     0,
#                     0,
#                     0,
#                     0,
#                     0,
#                     0,
#                     22,
#                     0
#                 )
#             print("message sended")

# def main(args=None):
#     rclpy.init(args=args)
#     sensor_sync = SensorSync()  # This line should correctly instantiate SensorSync
#     rclpy.spin(sensor_sync)


    
#     sensor_sync.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':

   
#     main()
