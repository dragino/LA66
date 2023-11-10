import requests

auth_token = '<TOKEN>'
devices_ip = '<IP>'
devtestid = '0111011111111111'

header = {'Authorization': 'Bearer ' + auth_token}
#查看数据需要将响应结果编码为JSON格式或者txt
def getdevices(ip):
    url = "http://" + ip + f":8080/api/devices?limit=10&applicationID=1"
    response = requests.get(url, headers=header)
    data = response.json()['result']
    for i in data:
        print(i['name'])
    #return data

    print(getdevices(devices_ip))

def deletedevices(ip):
    url = "http://" + ip + f":8080/api/devices/{devtestid}"
    response = requests.delete(url, headers=header)
    print(f"删除节点:DEVEUI={devtestid}")
    return response

#deletedevices(devices_ip)
