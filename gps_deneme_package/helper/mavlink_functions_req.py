import time
from pymavlink import mavutil,mavwp
import math
    

# Check last message was success 
def check_message_success(master):
    # Check that our DO_SET_MODE command was successful
    msg = master.recv_match(type="COMMAND_ACK",blocking=True)
    print(msg)
    if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Error setting mode")
        
        return False
    else:
        print("succes")
        return True

def set_parameter(mav, name, value, retries=3):
    got_ack = False

    # Parametreyi ayarlamak için MAVLink bağlantısına param_set gönder
    mav.param_set_send(name.upper(), float(value))

    while retries > 0 and not got_ack:
        retries -= 1
        # PARAM_ACK mesajını bekle
        msg = mav.recv_match(type='PARAM_VALUE', blocking=True)
        
        if msg is not None and msg.param_id.upper() == name.upper():
            got_ack = True
            print(f"Parameter {name} set successfully!")
            break

    if not got_ack:
        print(f"Failed to set parameter {name}")

def param_read(master,param_name):
    param_name_bytes = param_name.encode('utf-8')

    master.mav.param_request_read_send(0, 0, param_name_bytes, -1)

    msg = master.recv_match(type='PARAM_VALUE', blocking=True)
        
    if msg is not None and msg.param_id.upper() == param_name.upper():
        print(msg.param_value)
    # if msg is not None:
    #     print(msg)

def attitude_read(master):

    msg = master.recv_match(type='ATTITUDE', blocking=True)
        
    # print("yaw: ",math.degrees(msg.yaw))
    return msg.yaw

def is_armed(master):

    msg = master.recv_match(type='ARMED', blocking=True)
        
    # print("yaw: ",math.degrees(msg.yaw))
    return msg.yaw

def location_read(master):

    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        
    # print("yaw: ",math.degrees(msg.yaw))
    return msg.lat, msg.lon, msg.relative_alt, msg.hdg

def mode_read(master):

    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    
    for (number,mode_string) in mavutil.mode_mapping_acm.items():
        if mode_string == msg.custom_mode:
            return mode_string
    
    return None

def set_servo_pwm(master, servo_number, pwm_value):
        # Belirli bir servo kanalına PWM sinyali gönder
        master.mav.rc_channels_override_send(
            0,  # Sistem numarası
            0,  # Kanal maskesi (1'leri belirten bir bit dizisi, örneğin 00000001: Kanal 1)
            pwm_value,  # Kanal 1 PWM değeri
            0, 0, 0, 0, 0  # Diğer kanallar için PWM değerleri (boş bırakılmış)
        )


def set_home_location(master, latitude, longitude, altitude):
        # Ev konumunu ayarla
        master.mav.command_long_send(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # Komut: Set home location
            0,  # Parametre1: Kullanılmıyor, 0
            0,  # Parametre2: Kullanılmıyor, 0
            0,  # Parametre3: Kullanılmıyor, 0
            0,  # Parametre4: Kullanılmıyor, 0
            latitude,  # Parametre5: Ev konumu - Latitude
            longitude,  # Parametre6: Ev konumu - Longitude
            altitude  # Parametre7: Ev konumu - Altitude
        )


# Because of how ArduCopter works, need to look up a custom mode number
def get_custom_mode_number(mode):
    custom_mode_number = None
    # For ArduCopter, we use mode_mapping_acm, differs for ArduPlane or PX4
    # master.mode_mapping() should return the mapping needed
    for (number,mode_string) in mavutil.mode_mapping_acm.items():
        if mode_string == mode:
            custom_mode_number = number

    # Check if we actually found one
    if custom_mode_number is None:
        raise ValueError("Failed to find mode number for specified mode")
    
    return custom_mode_number

# Set mode via mavlink message
def set_mode_send(master,mode):
    # Create set mode message
    set_mode_message = mavutil.mavlink.MAVLink_command_long_message(
        1, # Target system
        1, # Target component
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, # Command
        0, # Confirmation counter
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, # Param 1 (Mode, for GUIDED, set to CUSTOM)
        get_custom_mode_number(mode), # Param 2 (Custom mode, GUIDED = 4 for ArduCopter)
        0, # Param 3 (Custom sub-mode, unused for ArduPilot)
        0, # Param 4 (Unused)
        0, # Param 5 (Unused)
        0, # Param 6 (Unused)
        0  # Param 7 (Unused)
    )
    master.mav.send(set_mode_message)

    #check_message_success(master)

# Define a function used to send commands in the future
# Unused params are left at 0
def send_command(master,command,confirmation,param1=0,param2=0,param3=0,param4=0,param5=0,param6=0,param7=0):
    """
    Send a COMMAND_LONG message to (sys,comp) = (1,1)
    """
    
    master.mav.command_long_send(
        1,1,
        command,
        confirmation,
        param1,
        param2,
        param3,
        param4,
        param5,
        param6,
        param7
        )
    #check_message_success(master)


def arm_vehicle(master):
    # Now we need to arm the vehicle
    print("Arming")
    send_command(master,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # Command
        0, # Confirmation
        1, # Param 1 (Arm)
        0, # Param 2 (Force)
        )
   # check_message_success(master)


    
    
def disarm_vehicle(master):
    # Now we need to arm the vehicle
    print("Disarming")
    send_command(master,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # Command
        0, # Confirmation
        0, # Param 1 (Arm)
        0, # Param 2 (Force)
        )
def do_change_speed(master,speed):
    send_command(
        master,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        1,
        speed)
    
def do_change_climb_descent_speed(master,climb_speed,descent_speed):
     master.mav.command_long_send(
            1,1,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            2,
            climb_speed,
            -1,
            0,
            0,
            0,
            0
            )
     master.mav.command_long_send(
            1,1,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            3,
            descent_speed,
            -1,
            0,
            0,
            0,
            0
            )
def do_change_air_ground_speed(master,air_speed,ground_speed):
     master.mav.command_long_send(
            1,1,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            0,
            air_speed,
            -1,
            0,
            0,
            0,
            0
            )
     master.mav.command_long_send(
            1,1,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1,
            ground_speed,
            -1,
            0,
            0,
            0,
            0
            )
def takeoff(master, altitude):
    # Command takeoff to 20m
    send_command(
    master,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # Command
    0, # Confirmation
    param7=altitude # Altitude (m)
    )
    #check_message_success(master)

def land(master, lat, lon, alt):
    # Command takeoff to 20m
    send_command(
        master,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # Command
        0, # Confirmation
        param4=float("NAN"),
        param5=lat,
        param6=lon,
        param7=alt # Altitude (m)
    )
   # check_message_success(master)


def set_position(master, lat, lon, alt,yaw):

    yaw = math.radians(yaw)
    # Wait for us to get to somewhere near 20m
    #msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)

    # Get current system time from the last message
    #time_pair = (time.time(), msg.time_boot_ms)

    # Define bitfields for ignoring velocity and acceleration
    ignore_velocity = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
       # | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )

    ignore_accel = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    )

    # Set the new position
    master.mav.set_position_target_global_int_send(
        0,#time_pair[1] + int(round((time.time() - time_pair[0]) * 1000)),
        1,
        1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        (ignore_velocity | ignore_accel),
        int(lat * (10 ** 7)),  # Latitude (degE7)
        int(lon * (10 ** 7)),  # Longitude (degE7)
        alt,  # Altitude
        0, 0, 0,  # Velocities
        0, 0, 0,  # Accelerations
        yaw,  # Yaw
        0  # Yaw rate
    )

    
def get_custom_mode_number(mode):
    mode_number = None
    # For ArduCopter, we use mode_mapping_acm, differs for ArduPlane or PX4
    # master.mode_mapping() should return the mapping needed
    for (number,mode_string) in mavutil.mode_mapping_acm.items():
        if mode_string == mode:
            mode_number = number

    # Check if we actually found one
    if mode_number is None:
        raise ValueError("Failed to find mode number for specified mode")
    
    return mode_number


def upload_mission(master, waypoints,home_point):
    # Görevi temiz
    wp = mavwp.MAVWPLoader()

    seq = 1
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    radius = 10

    # wp.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
    #                 master.target_component,
    #                 seq,
    #                 frame,
    #                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    #                 0, 0, 0, radius, 0, 0,
    #                 home_point[0],home_point[1],home_point[2]))
    # seq += 1
    master.waypoint_count_send(wp.count())   
    
    
    for (lat, lon, alt) in waypoints:                
        wp.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                    master.target_component,
                    seq,
                    frame,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 0, radius, 0, 0,
                    lat,lon,alt))    
          
        seq += 1                 

    master.waypoint_clear_all_send() 
    print(home_point[0], home_point[1], home_point[2])
    set_default_home_position(master, home_point[0], home_point[1], home_point[2]) 
    master.waypoint_count_send(wp.count())
    print(wp)                          

    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)             
        master.mav.send(wp.wp(msg.seq))                                                                      
        print('Sending waypoint {0}'.format(msg.seq))      
"""

Message encoding a waypoint. This message is emitted to announce
the presence of a waypoint and to set a waypoint on the system. The
waypoint can be either in x, y, z meters (type: LOCAL) or x:lat,
y:lon, z:altitude. Local frame is Z-down, right handed, global frame
is Z-up, right handed

target_system           : System ID (uint8_t)
target_component        : Component ID (uint8_t)
seq                     : Sequence (uint16_t)
frame                   : The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h (uint8_t)
command                 : The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs (uint8_t)
current                 : false:0, true:1 (uint8_t)
autocontinue            : autocontinue to next wp (uint8_t)
param1                  : PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters (float)
param2                  : PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds (float)
param3                  : PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise. (float)
param4                  : PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH (float)
x                       : PARAM5 / local: x position, global: latitude (float)
y                       : PARAM6 / y position: global: longitude (float)
z                       : PARAM7 / z position: global: altitude (float)
"""


def start_mission(master):
    # Görevi başlat
    master.mav.mission_start_send(0, 0)

def pause_mission(master):
    # Görevi duraklat
    master.mav.mission_pause_send(0, 0)

def resume_mission(master):
    # Görevi devam ettir
    master.mav.mission_resume_send(0, 0)

def clear_mission(master):
    # Görevi temizle
    master.waypoint_clear_all_send()

def monitor_mission_status(master):
    # Görev durumunu izle
    while True:
        msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
        if msg is not None:
            print(f"Reached waypoint {msg.seq}")


def get_next_waypoint(master):
        # Bir sonraki görev noktasını al
        while True:
            msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
            if msg is not None:
                next_wp = msg.seq + 1
                print(f"Next Waypoint: {next_wp}")
                return next_wp

# Send a mavlink SET_GPS_GLOBAL_ORIGIN message (http://mavlink.org/messages/common#SET_GPS_GLOBAL_ORIGIN), which allows us to use local position information without a GPS.
def set_default_global_origin(master,lat,lon,alt):
    master.mav.set_gps_global_origin_send(
        1,
        lat, 
        lon,
        alt
    )

# Send a mavlink SET_HOME_POSITION message (http://mavlink.org/messages/common#SET_HOME_POSITION), which allows us to use local position information without a GPS.
def set_default_home_position(master,lat,lon,alt):
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    master.mav.set_home_position_send(
        1,
        lat, 
        lon,
        0,
        x,
        y,
        z,
        q,
        approach_x,
        approach_y,
        approach_z
        )
    
#     )

def gps_input_send(master,lat,lon):
    master.mav.gps_input_send(
        0,
        0,
#         (mavutil.gps_input_ignore_flag.GPS_INPUT_IGNORE_FLAG_ALT|GPS_INPUT_IGNORE_FLAG_HDOP|GPS_INPUT_IGNORE_FLAG_VDOP|GPS_INPUT_IGNORE_FLAG_VEL_HORIZ|GPS_INPUT_IGNORE_FLAG_VEL_VERT|GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY
#         |GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY
#         ),
        1|4|8|16|32|128,
        0,
        0,
        3,
        lat,
        lon,
        6222,
        0.6,
        0.6,
        1,
        0,
        0,
        0.25,
        0.25,
        0,
        22
        
        )
    
# def gps_raw_send(master,message){
#     master.mav.gps_raw_int_send(




#     )
# }
def send_hil_gps(master,lat,lon,cog):
    
    result = master.mav.hil_gps_send(
        0,  
        2,  
        lat,
        lon,  
        628000,  
        20,  
        20,  
        0,  
        0,  
        0,  
        0,  
        cog,  
        22   
    )
    return result
#TODO
#CW CWW karar verme eksik
def set_yaw(master, yaw):

    lat, lon, alt, current_yaw = location_read(master)
    current_yaw = current_yaw / 100

    print(str(current_yaw) + " to " + str(yaw) + "    change: " + str(abs(current_yaw - yaw)))

    if 0 < yaw - current_yaw and yaw - current_yaw < 180:
        param_3 = 1
    else: 
        param_3 = -1 

    # Command takeoff to 20m
    send_command(
        master,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # Command
        0, # Confirmation
        param1=yaw,
        param2=40,
        param3=param_3,
        param4=0 # Altitude (m)
    )
    #check_message_success(master)

    while abs(math.degrees(attitude_read(master)) - yaw) > 10:
        time.sleep(0.3)

def send_target_locations(master, locations, home_point):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    clear_mission(master)
    print(" Clear any existing commands")

    target_loc = []


    for i in locations:
        target_x = i[0]
        target_y = i[1]

        location = (target_x,target_y,5)

        # if is_inside(uwb_geolocations, n_lat, n_lon):
        #     print('pooints in inside')
        #     target_loc.append(location)
        # else:
        #     print("Point is not inside.")
        
        target_loc.append(location)

    upload_mission(master,target_loc,home_point)

def send_target_locations_condition_yaw(master, locations, home_point):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    clear_mission(master)
    print(" Clear any existing commands")

    target_loc = []


    for i in locations:
        target_x = i[0]
        target_y = i[1]

        location = (target_x,target_y,5)

        # if is_inside(uwb_geolocations, n_lat, n_lon):
        #     print('pooints in inside')
        #     target_loc.append(location)
        # else:
        #     print("Point is not inside.")
        
        target_loc.append(location)

    upload_mission_condition_yaw(master,target_loc,home_point)

def calculate_bearing(lat1, lon1, lat2, lon2):
    # Latitude ve Longitude değerlerini radyan cinsine çevirme
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Δlon hesaplama
    delta_lon = lon2_rad - lon1_rad

    # Yatay hareket
    x = math.sin(delta_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - (math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))

    # Açıyı radian cinsinden alıp dereceye çevirme
    bearing_rad = math.atan2(x, y)
    bearing_deg = math.degrees(bearing_rad)

    # Açıyı 0-360 aralığına getirme
    bearing_deg = (bearing_deg + 360) % 360

    return bearing_deg
def upload_mission_condition_yaw(master, waypoints,home_point):
    # Görevi temiz
    wp = mavwp.MAVWPLoader()
    
    seq = 1
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    radius = 2

   
    _lat, _lon, _relative_alt, _hdg=location_read(master)

    _lat=_lat/1e7
    _lon=_lon/1e7
    w_lat,w_lon,w_alt=waypoints[0]
    _yaw=calculate_bearing(_lat,_lon,w_lat,w_lon)
    print(_yaw,_lat,_lon,w_lat,w_lon)
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                    master.target_component,
                    seq,
                    frame,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 5, radius, 0, 0,
                    _lat,_lon,w_alt/1000))
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                        master.target_component,
                        seq,
                        frame,
                        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                        0, 0,_yaw,30,-1,0,0,0,0 ))
    
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                    master.target_component,
                    seq,
                    frame,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 5, radius, 0, 0,
                    _lat,_lon,w_alt/1000))
            
    for i in range(len(waypoints)):
        
        lat, lon, alt = waypoints[i]

        wp.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                    master.target_component,
                    seq,
                    frame,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 0, radius, 0, 0,
                    lat,lon,alt))
        
        
        
        if i+1 < len(waypoints):
            lat2, lon2, alt2 = waypoints[i+1]
            yaw = calculate_bearing(lat, lon, lat2, lon2)
        
            wp.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                        master.target_component,
                        seq,
                        frame,
                        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                        0, 0,yaw,30,-1,0,0,0,0 ))
            
            wp.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                    master.target_component,
                    seq,
                    frame,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 5, radius, 0, 0,
                    lat,lon,alt))
            
            
        
        seq += 1                 

    master.waypoint_clear_all_send() 
    print(home_point[0], home_point[1], home_point[2])
    
    master.waypoint_count_send(wp.count())
    print(wp)                          

    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)             
        master.mav.send(wp.wp(msg.seq))                                                                      
        print('Sending waypoint {0}'.format(msg.seq)        )