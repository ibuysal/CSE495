
#-----CONENCTION STRINGS---------
IRIS_1_CONNECTION_STRING_1="127.0.0.1:14600"  #used in gps_publisher
IRIS_1_CONNECTION_STRING_2="127.0.0.1:14610"  #used in drone control
IRIS_1_CONNECTION_STRING_3="127.0.0.1:14620"
IRIS_1_CONNECTION_STRING_4="127.0.0.1:14630"  #used in eight_v2

IRIS_CONNECTION_STRING_1="127.0.0.1:14560"  # used in global_position_int

IRIS_CONNECTION_STRING_2="127.0.0.1:14570"   #used in gps_publisher

IRIS_CONNECTION_STRING_3='127.0.0.1:14580 ' # used in drone control
IRIS_CONNECTION_STRING_4='127.0.0.1:14540 '  #used local ned
IRIS_CONNECTION_STRING_5='127.0.0.1:14530 '  #used gimbal
#-----TOPIC NAMES---------------
IRIS_1_NAVSAT_TOPIC_NAME='/navsat_1'
IRIS_1_AIR_PRES_TOPIC_NAME='/air_pressure_1'
IRIS_1_GPS_NOISE_LEVEL_TOPIC_NAME='IRIS_1_gps_noise_level'
IRIS_1_GPS_WITH_NOISE_TOPIC_NAME='IRIS_1_gps_with_noise'
IRIS_1_GPS_NOISE_CARTESIAN_TOPIC_NAME='gps_noised_cartesian'
IRIS_1_ATTITUDE_TOPIC_NAME='iris_1_attitude'
IRIS_1_VFR_HUD_TOPIC_NAME='iris_1_vfr_hud'



IRIS_1_TRAJECTORY_POINTS_TOPIC_NAME='iris_1_x_y_z'
IRIS_1_GO_SIGNAL='iris_1_go_signal'
IRIS_1_XY='iris_1_xy'

IRIS_NAVSAT_TOPIC_NAME='/navsat'
IRIS_GPS_NOISE_LEVEL_TOPIC_NAME='IRIS_gps_noise_level'
IRIS_GPS_WITH_NOISE_TOPIC_NAME='IRIS_gps_with_noise'
IRIS_XY="iris_xy"

IRIS_ATTITUDE_TOPIC_NAME='iris_attitude'
IRIS_VFR_HUD_TOPIC_NAME='iris_vfr_hud'
IRIS_IMAGE_TOPIC_NAME='camera/image'
IRIS_IMAGE_RECTANGLE_CENTER_TOPIC_NAME='red_drone_bounding_box_center'
IRIS_GLOBAL_POSITION_INT='iris_global_position_int'
IRIS_LOCAL_POSITION_INT='iris_local_ned'

SENSOR_DISTANCE_TOPIC_NAME='gps_sensor_distance'
MOVEMENT_PARAMS='movement_params'

TAKEOFF_SIGNAL='takeoff_signal'

#-------IRIS_1 SPEES-----

IRIS_1_CLIMB_DESCENT_SPEED=2 #m/s
IRIS_1_GROUND_AIR_SPEED=2   #m/s
IRIS_1_ACCEL=200 #cm/s'2


#-------PARAM NAMES-------

ACCEL="WPNAV_ACCEL" #CM/S'2
ACCEL_Z="WPNAV_ACCEL_Z"  #CM/S'2
EXTRA_2="SR0_EXTRA2"
EXTRA_1="SR0_EXTRA1"
EXTRA_3="SR0_EXTRA3"
SR0_EXT_STAT="SR0_EXT_STAT"

# <lens>
#         <intrinsics>
#           <fx>350</fx>
#           <fy>350</fy>
#           <cx>540</cx>
#           <cy>360</cy>
#           <s>0</s>
#         </intrinsics>
#         </lens>