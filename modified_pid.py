from dronekit import connect,VehicleMode,LocationGlobalRelative,mavutil
import time
from dronekit_sitl import SITL
import math
import sympy
import matplotlib.pyplot as plt
# Connect to the Vehicle (SITL runs on default IP: 127.0.0.1, Port: 14550)
connection_string = '127.0.0.1:14550'  # Default SITL connection
print("Connecting to vehicle on:", connection_string)
vehicle = connect(connection_string, wait_ready=True)


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
        time.sleep(1)

arm_and_takeoff(2)
vehicle.mode = VehicleMode("GUIDED")

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.02)

def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # Radius of the Earth in km
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c*1000 # Distance in kilometers

def bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    delta_lon = lon2 - lon1
    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    initial_bearing = math.atan2(x, y)
    initial_bearing = (math.degrees(initial_bearing) + 360) % 360
    return initial_bearing

#lat3,lon3 =  -35.36205354,149.16447253 #up right
#lat3,lon3 =  -35.36186808,149.16612687 
lat3,lon3= -35.36308193,149.16519115  #20metres down the road
#lat3,lon3=-35.36326212,149.16523751  #home
kp=1
ki=0.001
kd=0.4
loc_c=vehicle.location.global_frame
lati=loc_c.lat
loni=loc_c.lon
e2=haversine(lat3,lon3,lati,loni)
distance=haversine(lat3,lon3,lati,loni)
error=0.001


integral_1=0
dt = None

v_plot = []
e_plot = []

def pid(kp,ki,kd,error,dt):
    proportional = kp*error
    global integral_1
    loci=vehicle.location.global_frame
    latc=loci.lat
    lonc=loci.lon
    k=1
    if haversine(lati,loni,latc,lonc)>distance:
        k=-1
    #integral
    rocofs=vehicle.velocity
    vx,vy,vz = rocofs
    differential = float('nan')
    differential=k*kd*(-math.sqrt(vx*vx+vy*vy))
    
    
    
    # if differential>0:
    #      k=-k
    # if(error<0.15):
    #      k=-k
    integral_1+=k*error*dt
    integral=ki*integral_1

    #print("integral:",integral)
    #print(k)
    #print("error:",error)
    print("differential:",differential)
    velocity = proportional+integral+differential
    # print(dt,e2-e1)
    return velocity

#v=7
# list1=[(lat1,lon1),(lat2,lon2),(lat3,lon3),(lat4,lon4),(lat1,lon1)]
# for i in list1:
while True:
    if dt == None:
        dt = 0.001

    loc1=vehicle.location.global_frame
    lat_=loc1.lat
    lon_=loc1.lon
    latk,lonk=lat3,lon3
    t1=time.time()
    # k=1
    # if error<0.5:
    #     k= -1*k
    error=haversine(latk,lonk,lat_,lon_)
    theta = math.radians(bearing(lat_,lon_,latk,lonk))
    if error<=10:
        v  = pid(kp,ki,kd,error,dt)
        
    else:
        v=5
    v_plot.append(v)
    e_plot.append(error)
    vx = v*math.cos(theta)
    vy = v*math.sin(theta)
    send_ned_velocity(vx,vy,0,1)
    t2=time.time()
    dt=t2-t1
    # if error<0:
    #     error=-error
    e2=error
    if error<0.05:
        send_ned_velocity(0,0,0,1)
        print("PID is spot on bitch!!")
        break


plt.plot(range(len(v_plot)), v_plot, marker='o', linestyle='-', color='b')
plt.plot(range(len(e_plot)), e_plot, marker='o', linestyle='-', color='g')

# Add labels and title
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('Plot of List v with Index')
# Show the plot
plt.show()