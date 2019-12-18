import roslibpy
import matplotlib.pyplot as plt
import tkinter
import matplotlib
matplotlib.use('TkAgg')

def get_service(client):
    service = roslibpy.Service(client, 'get_telemetry', 'clever/GetTelemetry')
    request = {'frame_id':'aruco_map'}
    print('Calling service...')
    result = service.call(request)
    return  result


if __name__ == "__main__":
    client = roslibpy.Ros(host='192.168.1.189', port=9090)
    client.run()

    data1 = []
    data2 = []
    for i in range(0, 50):
        res = get_service(client)
        data1.append(float(res['x']))
        data2.append(float(res['y']))
    client.terminate()
    plt.plot(data1, data2, 'o', color='blue')
    plt.show()