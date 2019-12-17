# import  paramiko
# import time
#
#
# if __name__ == "__main__":
#     client = paramiko.SSHClient()
#     client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
#     client.connect(hostname='192.168.1.189', username="pi", password="raspberry", port=22)
#     channel = client.get_transport().open_session()
#     channel.get_pty()
#     channel.settimeout(5)
#     channel.exec_command('python test.py')
#     finish = channel.recv(1024).decode('utf-8')
#     try:
#         while True:
#             time.sleep(1)
#             finish = channel.recv(1024).decode('utf-8')
#             print(finish)
#     except KeyboardInterrupt:
#         pass
#     channel.close()

#
# import roslibpy
#
# client = roslibpy.Ros(host='192.168.1.189', port=9090)
# client.run()
# print('Is ROS connected?', client.is_connected)
# client.terminate()

import roslibpy

client = roslibpy.Ros(host='192.168.1.189', port=9090)
client.run()

service = roslibpy.Service(client, 'get_telemetry', 'clever/GetTelemetry')
#request = roslibpy.ServiceRequest()

request = {'frame_id':'aruco_map'}

print('Calling service...')
result = service.call(request)
print(result)

client.terminate()