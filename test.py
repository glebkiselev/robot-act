import rospy
from clever import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')  # 

# 

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)


z = 1.5  # 
tolerance = 7.0

# 
start = get_telemetry()
print start.x, start.y, start.z

rospy.sleep(2)
# 
#print navigate(z=z, speed=0.5, frame_id='body', auto_arm=True)

xx=[]
yy=[] 
while True:
    telemetry = get_telemetry(frame_id='aruco_map')
    print telemetry.x, telemetry.y, telemetry.yaw
   
   
    if telemetry.y > tolerance:
        # 
        break
    rospy.sleep(0.2)
print "> 1.0" 
