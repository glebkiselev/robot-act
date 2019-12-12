from utils import *


# 9.72 for circle in any direction if speed is 30,30
# 3 for 1 meter in any direction if speed is 30, 30

def rotate_move_dist(dir, dist):
    if dir == 'Right' or dir == 'Left':
        goal_time = 9.72172 * float(dist) / 360.0
    else:
        goal_time = 3.0 * float(dist)

    start_time = time.time()
    print("start time is %s" % str(start_time))
    curr_time = start_time

    while curr_time - start_time <= goal_time:
        try:
            func = globals()['MotorAB_Direction_' + dir]
            func(30, 30)
            time.sleep(0.1)
            curr_time = time.time()
            # print("curr time is %s " % str(curr_time))
        except KeyboardInterrupt:
            MotorAB_Brake()
            break
    print("goal time was %s" % str(goal_time))
    print("move time was %s" % str(time.time() - start_time))
    return True


if __name__ == "__main__":

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(dest="direct", nargs='?')
    parser.add_argument(dest="dist", nargs='?', default='0')
    args = parser.parse_args()
    # if rotate dist is in degrees 0-360
    # if move forward or backward dist is in meters
    direct = args.direct
    dist = args.dist
    if direct != 'Suck' and direct != 'Unsuck':
        res = rotate_move_dist(direct, dist)
    else:
        func = globals()['MotorAB_Direction_' + direct]
        res = func()
    print(res)


