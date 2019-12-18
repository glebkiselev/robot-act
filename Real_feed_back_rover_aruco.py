
import sys
#import serial

import time
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import cubic_spline_planner
#from sklearn.metrics import mean_squared_error
import timeit

import rospy
from clever import srv
from std_srvs.srv import Trigger
#from casper import serwrite

rospy.init_node('fli')  # 
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
# 

def getT(flag=0):
    telemetry = get_telemetry(frame_id='aruco_map')
    if(flag):
        print telemetry.x, telemetry.y, telemetry.yaw, (telemetry.vx**2+telemetry.vy**2)**(1/2)
    return telemetry

Kp = 4.0  # speed propotional gain
# steering control parameter
KTH = 1.0
KE = 0.5

N_IND_SEARCH = 10
MAX_STEER = math.radians(15.0)
dt = 0.05  # [s]
dT = 10# [s]
L = 0.2  # [m]

show_animation = True
#  show_animation = False

GOAL_DIS = 0.5  # goal distance
STOP_SPEED = 0.5 / 3.6    # stop speed

DX = 0
DY = 0
DAngle = 0

def get_service():
    import roslibpy

    client = roslibpy.Ros(host='192.168.1.189', port=9090)
    client.run()

    service = roslibpy.Service(client, 'get_telemetry', 'clever/GetTelemetry')

    request = {'frame_id':'aruco_map'}

    print('Calling service...')
    result = service.call(request)

    client.terminate()
    return  result

def getnewtelem(n=10,sh=0):
    x=0;y=0;yaw=0;vx=0;vy=0
    fr=0
    xmin=0;ymin=0;yawmin=0;vxmin=0;vymin=0
    xmax=0;ymax=0;yawmax=0;vxmax=0;vymax=0
    for i in range(0,n):
        telem=get_service()
        x+=float(telem['x'])
        y+=float(telem['y'])
        yaw+=float(telem['yaw'])
        vx+=float(telem['vx'])
        vy+=float(telem['vy'])
        if(fr==0):
            fr=1
            xmin=xmax=float(telem['x'])
            ymin=ymax=float(telem['y'])
            yawmin=yawmax=float(telem['yaw'])
            vxmin=vxmax=float(telem['vx'])
            vymin=vymax=float(telem['vy'])
        else:
            
            xmin=min(float(telem['x']),xmin)
            ymin=min(float(telem['y']),ymin)
            yawmin=min(float(telem['yaw']),yawmin)
            vxmin=min(float(telem['vx']),vxmin)
            vymin=min(float(telem['vy']),vymin)
            xmax=max(float(telem['x']),xmax)
            ymax=max(float(telem['y']),ymax)
            yawmax=max(float(telem['yaw']),yawmax)
            vxmax=max(float(telem['vx']),vxmax)
            vymax=max(float(telem['vy']),vymax)
        
    if(n>2):
        x-=xmin+xmax
        y-=ymin+ymax
        yaw-=yawmin+yawmax
        vx-=vxmin+vxmax
        vy-=vymin+vymax
        x=x/(n-2)
        y=y/(n-2)
        yaw=yaw/(n-2)
        vx=vx/(n-2)
        vy=vy/(n-2)
    else:
        x=x/(n)
        y=y/(n)
        yaw=yaw/(n)
        vx=vx/(n)
        vy=vy/(n)
        
    telem['x']=x
    telem['y']=y
    telem['yaw'] = yaw
    telem['vx']=vx
    telem['vy']=vy
    
    return telem


def radtograd(angle):
    return (angle/math.pi)*180

class State:

    def __init__(self, x=0., y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.xold = 0
        self.y = y
        self.yold = 0
        self.yaw = yaw
        self.yawold = 0
        self.v = v
        self.w = 0.2
        self.r = 0.051
        self.pred=[0.0,0.0]

def update(state, a, delta,b=0):
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    time.sleep(dt*8/3)

    
    telem=getnewtelem() #getT(1)
    #print telem.x, telem.y, math.acos(0-math.cos(telem.yaw))*(telem.yaw/abs(telem.yaw))
    if(b): print"delta: ",
    if(b): print delta    
    if(b): print"yaw:  ",
    if(b): print radtograd(telem['yaw'])
    #if(b): print"dx: ",
    #if(b): print(state.x - state.xold)
    #if(b): print"dy: ",
    #if(b): print(state.y-state.yold)
    #if(b): print"yawn: ",
    #if(b): print radtograd(state.yaw)

#    if(((telem.x-state.x)**2+(telem.y-state.y)**2)**(1/2)<0.1):
#        return state
    state.xold = state.x
    state.x=telem.x #+DX #OTNOSITELNIE
    state.yold = state.y
    state.y=telem.y # +DY #OTNOSITELNIE
    state.yawold = state.yaw
    #state.yaw=math.atan2(state.y-state.yold, state.x - state.xold) #
    state.yaw=(math.acos(0-math.cos(telem.yaw)))*(telem.yaw/abs(telem.yaw))
    state.v=(telem.vx**2+telem.vy**2)**(1/2)

    print state.x, state.y
    #print("delta: ")
    #print delta    
    #print("yaw:  ")
    #print radtograd(telem.yaw*(-1))
    #if(b): print"dx: ",
    #if(b): print(state.x - state.xold)
    #if(b): print"dy: ",
    #if(b): print(state.y-state.yold)
    #if(b): print"yawn: ",
    #if(b): print radtograd(state.yaw)
    
#    if(telem.x<3.9 or telem.y<3.8):
#        _=1/0 
    
    return state


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle



"""
def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind
"""
def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]
    
    #dx = [state.x - icx for icx in cx]
    #dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1
        
    if pind >= ind:
        ind = pind

    return ind, mind

def calc_ref_trajectory(state, cx, cy, cyaw, pind):
    
    ind, e = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    return ind, e

def check_goal(state, goal, tind, nind):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.sqrt(dx ** 2 + dy ** 2)

    if (d <= GOAL_DIS):
        isgoal = True
    else:
        isgoal = False

    if abs(tind - nind) >= 5:
        isgoal = False

    if (abs(state.v) <= STOP_SPEED):
        isstop = True
    else:
        isstop = False

    if isgoal:
        return True

    return False


#                                v      -  yaw  cyaw  k   e
def rear_wheel_feedback_control(state, cx, cy, cyaw, ck, preind):
    ind, e = calc_nearest_index(state, cx, cy, cyaw, preind)

    k = ck[ind]#0.35#
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    omega = v * k * math.cos(th_e) / (1.0 - k * e) - \
        KTH * abs(v) * th_e - KE * v * math.sin(th_e) * e / th_e

    if th_e == 0.0 or omega == 0.0:
        return 0.0, ind

    delta = math.atan2(L * omega / v, 1.0)
    #  print(k, v, e, th_e, omega, delta)

    return delta, ind

def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal):

    T = 500.0  # max simulation time
    goal_dis = 0.3
    stop_speed = 0.05
    flag = True
    goal = [cx[-1], cy[-1]]
    state = State(x=cx[0], y=cy[0], yaw=DAngle, v=0.0)

    timet = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    xr=[]
    yr=[]
    d = []
    a=[]
    yawr=[]
    goal_flag = False
    #target_ind = calc_nearest_index(state, cx, cy, cyaw, 0)
    
    target_ind =0
    cyaw = smooth_yaw(cyaw)
    
        #target_ind =calc_ref_trajectory(state, cx, cy, cyaw, target_ind)
    cyaw = smooth_yaw(cyaw)
    while 1:
        di, target_ind = rear_wheel_feedback_control(state, cx, cy, cyaw, ck, target_ind)
        #print(timeit.default_timer() - start_time)
        print ("TRAJECTORY")
        print cx[target_ind], cy[target_ind]
        ai = PIDControl(speed_profile[target_ind], state.v)
        state = update(state, ai, di)


        if abs(state.v) <= stop_speed:
            target_ind += 1

        timet = timet + dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        
        if check_goal(state, goal, target_ind, len(cx)):
            print("Goal")
            #serwrite([0,0,0,0.4],1)
            time.sleep(0.05)
            #serwrite([0,0,0,0.5],1)
            break
        
        #print(state.v)
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(timet)
        d.append(round(di,2))
        a.append(round(ai,2))
        xr.append(cx[target_ind])
        yr.append(cy[target_ind])
        yawr.append(cyaw[target_ind])
        if target_ind % 1 == 0 and show_animation:
            _=0
            plt.cla()
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            #plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2)) +
            #          ",target index:" + str(target_ind))
            plt.pause(0.0001)

    return t, x, y, yaw, v, goal_flag, xr, yr, yawr, d, a

def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-2] = 0.0
    speed_profile[-1] = 0.0
    return speed_profile

def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw

def main():
    
    print("rear wheel feedback tracking start!!")
    #ser = serial.Serial('/dev/tty.usbmodem41332401', 9600, timeout=0)
    #ax = [0.0, 6.0, 12.5, 5.0, 7.5, 3.0, -1.0]
    #ay = [0.0, 0.0, 5.0, 6.5, 3.0, 5.0, -2.0]
    a = 1  # a = 2, b = 2 krug; a = 1, b = 2 lissazu;  a = 2, b = 1 parabala;
    b = 2
    d = 0.5 * math.pi
    telem=getT(1)
    x0 = telem.x
    y0 = telem.y
    ax = [math.sin(a * t + d)*1 for t in np.arange(0., 2. * math.pi, 0.1)]
    ay =  [math.sin(b * t)*1 for t in np.arange(0.,2. * math.pi, 0.1)]
    
    #ax = [0.0, 0.0, 1.0, 1.5, 2.0, 2.0]
    #ay = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5]
#    ay = [0.0, 0.1, 0.1, 0.1, 0.1, 0.1]
    ax = [x0,6.5, 6.5]
    ay = [y0, 4.5,5.0]
    #goal = [ax[-1], ay[-1]]
    global DX
    DX=x0-ax[0]
    global DY
    DY=y0 -ay[0]
    global DAngle
    lstx=[]
    for a in ax:
        lstx.append(DX+a)
    print lstx
    lsty=[]
    for b in ay:
        lsty.append(DY+b)

    goal = [lstx[-1], lsty[-1]]
#    ax[0]=x0 #OTN
#    ay[0]=y0 #OTN
    DAngle =math.acos(0-math.cos(telem.yaw))*(telem.yaw/abs(telem.yaw))
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        lstx, lsty, ds=0.1)
    target_speed = (3.6) / 3.6
    #print(x0+goal[0]-ax[0],y0+goal[1]-ay[0])
    sp = calc_speed_profile(cx, cy, cyaw, target_speed)
    #print (sp)
    t, x, y, yaw, v, goal_flag, xr, yr, yawr, d, a = closed_loop_prediction(
        cx, cy, cyaw, ck, sp, goal)
    
    # delta_d = d#.round(2)
    # delta_v = v
    # print (len(d))
    # strok = str(d)[1:-1]
    # #strok = ''.join(map(str, strok))
    # #strok = ''.join(strok)
    # strok = '11'+ ','+str(len(d))+','+ strok
    # print(strok)
    # #ser.write(strok.encode())

    # #print (len(delta_d))
    # out_str="int traektor_size="+str(len(delta_d))+";\nfloat traektor[]={ "
    # for i in delta_d:
    #     out_str+=str(i)+", "
    # #out_str=out_str[0:]+"};\nint v_size="+str(len(delta_d))+";\nfloat velos[]={ "
    # #for i in delta_v:
    #     #out_str+=str(i)+", "
    # out_str = out_str[0:] + "};"
    #     #print(out_str)
    # f=open("out_line_trajektory.txt", "w")
    # f.write(out_str)
    # f.close()

    # Test
    #assert goal_flag, "Cannot goal"
    #print(mean_squared_error(x[1:], xr))
    #print(mean_squared_error(y[1:], yr))
    #print(mean_squared_error(yaw[1:], yawr))
    
    #print(yaw)
    #print(yawr)
    #while True:
        #line = ser.readline().decode("utf-8").replace('"', '').strip('\n').strip('\r')
        #print(line)
    if show_animation:
        _=0
        plt.close()
        plt.subplots()
        #plt.plot(ax, ay, "xb", label="input")
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
           
        #plt.subplots(1)
        #plt.plot(s, [np.rad2deg(iyaw) for iyaw in cyaw], "-r", label="yaw")
        #plt.grid(True)
        #plt.legend()
        #plt.xlabel("line length[m]")
        #plt.ylabel("yaw angle[deg]")

        #plt.subplots(1)
        #plt.plot(s, ck, "-r", label="curvature")
        #plt.grid(True)
        #plt.legend()
        #plt.xlabel("line length[m]")
        #plt.ylabel("curvature [1/m]")

            ##plt.subplots()
            ##plt.plot(t[:-109], d, "-r", label="angle")
            ##plt.grid(True)
            ##plt.xlabel("Time [s]")
            ##plt.ylabel("Rad")
        
        plt.show()


if __name__ == '__main__':
    while(1):
        try:
            #serwrite([0,0,0,0.9],1)
            #time.sleep(0.5)
            main()
        except:
            break
        finally:
            #serwrite([0,0,0,0.4])
            time.sleep(0.050)
            #serwrite([0,0,0,0.5])
    


