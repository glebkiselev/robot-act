import paramiko
import time
robots = {
'irod' : ['192.168.1.219', 'pi', 'raspberry', 22],
'axil' : ['192.168.1.98', 'pi', 'raspberry', 22]
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
    #channel.send(robot[2]+'\n')
    finish = channel.recv(1024).decode('utf-8')
    while not 'True' in finish[-6:]:
        time.sleep(1)
        finish = channel.recv(1024).decode('utf-8')
    channel.close()
    return finish



if __name__ == "__main__":
    plan = [['irod', 'Forward', 2], ['irod', 'Right', 90], ['irod', 'Suck', ''],  ['irod', 'Forward', 1], ['irod', 'Right', 90], ['irod', 'Forward', 1], ['irod', 'Unsuck', '']]
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