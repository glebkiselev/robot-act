import roslibpy

client = roslibpy.Ros(host='192.168.1.189', port=9090)
client.run()

service = roslibpy.Service(client, 'get_telemetry', 'clever/GetTelemetry')

request = {'frame_id':'aruco_map'}

print('Calling service...')
result = service.call(request)
print(result['yaw'])

client.terminate()