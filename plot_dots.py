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
    elem1 = 0
    elem2 = 0
    for elem in data1:
        elem1+=elem
    elem1 = elem1 / 50
    for elem in data2:
        elem2+=elem
    elem2 = elem2 / 50

    fig, axs = plt.subplots(2)
    fig.suptitle('wdfsgrfs')
    axs[0].plot(elem1, elem2, 'o', color = 'red')
    axs[1].plot(data1, data2, 'o', color = 'blue')
    plt.show()
