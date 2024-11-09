from dronekit import connect,VehicleMode,LocationGlobalRelative
import time
import math
from dronekit_sitl import SITL

# Connect to the Vehicle (SITL runs on default IP: 127.0.0.1, Port: 14550)
connection_string = '127.0.0.1:14550'  # Default SITL connection
print("Connecting to vehicle on:", connection_string)
vehicle = connect(connection_string, wait_ready=True)

import math

def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # Radius of the Earth in km
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance=R*c
    return R * c  # Distance in kilometers



def arm_and_takeoff(aTargetAltitude):

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break  
        time.sleep(5)

arm_and_takeoff(10)
time.sleep(5)
vehicle.airspeed=10
vehicle
vehicle.mode = VehicleMode("GUIDED")

# Set the target location in global-relative frame
a_loc= LocationGlobalRelative(-35.36357088,149.16625631,10)
b_loc = LocationGlobalRelative(-35.36354843,149.16734405,10)
c_loc = LocationGlobalRelative(-35.36442424,149.16733028,10)
d_loc = LocationGlobalRelative(-35.36445793,149.16624254,10)

list1 =[a_loc,b_loc,c_loc,d_loc,a_loc]
for i in list1:
    a,b,c = i.lat,i.lon,i.alt
    vehicle.simple_goto(i)
    time.sleep(5)
    while True:
        location = vehicle.location.global_frame
        lat=location.lat
        lon=location.lon
        alt=location.alt
        distance = haversine(a,b,lat,lon)
        print("distance:",distance)
        time.sleep(2)
        if distance<=0.001:
            print("Reached wp",list1.index(i)+1," Successfully!")
            break
        time.sleep(2)

# vehicle.simple_goto(a_loc)

# while True:
#     location = vehicle.location.global_frame
#     lat = location.lat
#     lon = location.lon
#     alt = location.alt
#     distance =haversine(a,b,lat,lon)
#     if distance<=1:
#         print("Reached Successfully!")
#         break


# vehicle.simple_goto(a_loc)
# time.sleep(10)
# vehicle.simple_goto(b_loc)
# time.sleep(10)
# vehicle.simple_goto(c_loc)
# time.sleep(10)
# vehicle.simple_goto(d_loc)
# time.sleep(10)