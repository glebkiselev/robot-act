import math
import os
import platform
import sys
import re
from copy import copy

import paramiko
import time
robots = {
'irod' : ['192.168.1.219', 'pi', 'raspberry', 22],
'axil' : ['192.168.1.50', 'pi', 'raspberry', 22]
}

aruko = {
    'above':3.0,
    'above-right': -2.2,
    'above-left': 2.2,
    'left': 1.59,
    'right': -1.59,
    'below': 0.0,
    'below-left': 0.96,
    'below-right': -0.96,
}


directions = {
    'above':{
        'above-right': ['Right', 45],
        'above-left': ['Left', 45],
        'left': ['Left', 90],
        'right': ['Right', 90],
        'below': ['Left', 180],
        'below-left': ['Left', 135],
        'below-right': ['Right', 135],

    },
    'below': {
        'above-right': ['Left', 135],
        'above-left': ['Right', 135],
        'left': ['Right', 90],
        'right': ['Left', 90],
        'above': ['Left', 180],
        'below-left': ['Right', 45],
        'below-right': ['Left', 45],

    },
    'right': {
        'above-right': ['Left', 45],
        'above-left': ['Left', 135],
        'left': ['Left', 180],
        'above': ['Left', 90],
        'below': ['Right', 90],
        'below-left': ['Right', 135],
        'below-right': ['Right', 45],

    },
    'left': {
        'above-right': ['Right', 135],
        'above-left': ['Right', 45],
        'above': ['Right', 90],
        'right': ['Left', 180],
        'below': ['Left', 90],
        'below-left': ['Left', 45],
        'below-right': ['Left', 135],

    },
    'above-right': {
        'above': ['Left', 45],
        'above-left': ['Left', 90],
        'left': ['Left', 135],
        'right': ['Right', 45],
        'below': ['Right', 135],
        'below-left': ['Left', 180],
        'below-right': ['Right', 90],

    },
    'above-left': {
        'above-right': ['Right', 90],
        'above': ['Right', 45],
        'left': ['Left', 45],
        'right': ['Right', 135],
        'below': ['Left', 135],
        'below-left': ['Left', 90],
        'below-right': ['Left', 180],

    },
    'below-right': {
        'above-right': ['Left', 90],
        'above-left': ['Left', 180],
        'left': ['Right', 135],
        'right': ['Left', 45],
        'below': ['Right', 45],
        'below-left': ['Right', 90],
        'above': ['Left', 135],

    },
    'below-left': {
        'above-right': ['Left', 180],
        'above-left': ['Right', 90],
        'left': ['Right', 45],
        'right': ['Left', 135],
        'below': ['Left', 45],
        'above': ['Right', 135],
        'below-right': ['Left', 90],

    },

}

def client_creater(robot):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(hostname=robot[0], username=robot[1], password=robot[2], port=robot[3])
    return client


def command_execute(client, command):
    channel = client.get_transport().open_session()
    channel.get_pty()
    channel.settimeout(5)
    channel.exec_command('python robot_labs/move_rotate.py '+command)
    finish = channel.recv(1024).decode('utf-8')
    while not 'True' in finish[-6:]:
        time.sleep(1)
        finish = channel.recv(1024).decode('utf-8')
    channel.close()
    return finish


if __name__ == "__main__":
    if platform.system() != 'Windows':
        delim = '/'
        projectPath = delim + ''.join([el + delim for el in os.getcwd().split(delim)[1:-1]])
    else:
        delim = '\\'
        projectPath = ''.join([el + delim for el in os.getcwd().split(delim)[:-1]])

    if not 'solution.txt' in os.listdir(os.getcwd()):
        sys.path.append(projectPath + 'map-spatial')
        import test2
        main = getattr(test2, 'main')
        solution = main([])
        with open('solution.txt', 'w', encoding='utf-8') as sol:
            data = [(el[1], el[3].name, el[4][0]['x'], el[4][0]['y'], el[4][1]) for el in solution]
            for element in data:
                sol.write(str(element)+'\n')
    else:
        with open('solution.txt', 'r', encoding='utf-8') as sol:
            data = []
            while True:
                d = sol.readline()
                if len(d) == 0:
                    break
                else:
                    splitted = d.split("'")
                    pose = splitted[4].split(',')
                    x = float(pose[1])
                    y = float(pose[2])
                    data.append((splitted[1].strip(), splitted[3].strip(), x, y, splitted[5].strip()))

    def tokenizer(data):
        for el in data:
            yield el
    plan = []
    flag = False
    #todo take 1 direction and place from file
    prev_dir = 'above'
    prev_place = [130.0, 130.0]
    my_token = tokenizer(data)
    while not flag:
        try:
            element = next(my_token)
            if element[1] == 'I' or element[1] == 'ag1':
                name = 'axil'
            else:
                name = 'irod'
            if element[0] == 'rotate':
                direction = directions[prev_dir][element[4]]
                plan.append([name, direction[0], direction[1], aruko[element[4]]])
                prev_dir = element[4]
            elif element[0] == 'move':
                path = math.sqrt((element[3]-prev_place[1])**2 + (element[2]-prev_place[0])**2)
                plan.append([name, 'Forward', round(path/100, 2), aruko[prev_dir]])
                prev_place = [element[2], element[3]]
            elif element[0] == 'pick-up':
                new_dir = [dir for dir, key in directions[prev_dir].items() if key[1] == 180][0]
                plan.append([name, 'Left', 180, aruko[new_dir]])
                plan.append([name, 'Pickup', '', aruko[new_dir]])
                plan.append([name, 'Right', 180, aruko[prev_dir]])
            elif element[0] == 'Clarify' or element[0] == 'Abstract':
                continue
            else:
                new_dir = [dir for dir, key in directions[prev_dir].items() if key[1] == 180][0]
                plan.append([name, 'Left', 180, aruko[new_dir]])
                plan.append([name, 'Stack', '', aruko[new_dir]])
                plan.append([name, 'Right', 180, aruko[prev_dir]])
        except StopIteration:
            flag = True
    prev_el = []
    next_index = None
    index = 0
    for el in copy(plan):
        if next_index:
            index = next_index
        if prev_el:
            if prev_el[0] == el[0]:
                name = el[0]
                if (prev_el[1] == 'Left' and el[1] == 'Left') or (prev_el[1] == 'Right' and el[1] == 'Right'):
                    plan.pop(index-1)
                    try:
                        # coze enum move up
                        plan.pop(index-1)
                    except IndexError:
                        pass
                    new_angle = 360 - prev_el[2] - el[2]
                    if el[1] == "Left":
                        dir = 'Right'
                    else:
                        dir = 'Left'
                    plan.insert(index-1, [name, dir, new_angle])
                    next_index = index
                elif (prev_el[1] == 'Left' and el[1] == 'Right') or  (prev_el[1] == 'Right' and el[1] == 'Left') :
                    plan.pop(index-1)
                    try:
                        #coze enum move up
                        plan.pop(index-1)
                    except IndexError:
                        pass
                    if el[2] > prev_el[2]:
                        new_angle = el[2] - prev_el[2]
                        dir = 'Right'
                    else:
                        new_angle = prev_el[2] - el[2]
                        dir = 'Left'
                    if new_angle != 0:
                        plan.insert(index-1, [name, dir, new_angle])
                        next_index = index
                    else:
                        next_index = index-1
                else:
                    next_index = index+1
                prev_el = el
        else:
            prev_el = el
            next_index = index+1

    client = None
    prev_name = ''
    for element in plan:
        finish = False
        name = element[0]
        if not prev_name or prev_name != name:
            if prev_name:
                client.close()
            client = client_creater(robots[name])
        finish = command_execute(client, element[1] + ' ' + str(element[2]))
        print(finish)
        prev_name = name
    client.close()