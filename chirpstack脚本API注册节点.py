#使用API通过requests库用POST请求在chripstack创建节点
import requests
import json
import time

device_ip = '<IP>'
device_profile_id = "<DEVICE-UUID>"
application_id = 5
app_key = "11111111111111111111111111111111"
gen_app_key = "11111111111111111111111111111111"

f = open("Keys_WTS10_20220511_EU868.txt","r")

#跳过第一行
readline1 = f.readline()
count = 0
for i in readline1:
    count += 1
    readline1 = f.readline()
    if readline1[1:2]=='':
        print(f'注册完毕,共{count-1}台节点')
        break
    dev = readline1[12:28]
    appkey = readline1[46:78]
    #API KEY
    auth_token = '<TOKEN>'    #POST headers
    header = {'Authorization': 'Bearer ' + auth_token}
    #创建节点,chripstack与TTN不同先需要通过DEVEUI创建节点后，再给节点设置APPKEY
    data1 = {
      "device": {
        "applicationID": application_id,
        "description": "test",
        "devEUI": dev,
          "deviceProfileID": device_profile_id,
        "isDisabled": False,
        "name": dev,
        "referenceAltitude": 0,
        "skipFCntCheck": False,
        "tags": {},
        "variables": {}
      }
    }
    #给节点设置APPKEY，与WebUI不同使用API时APPKEY为NWKKEY
    data2 ={
      "deviceKeys": {
        "appKey": app_key,
        "devEUI": dev,
        "genAppKey": gen_app_key,
        "nwkKey": appkey
      }
    }
    #需要摆字典转换为JSON格式
    data3 = json.dumps(data1)
    data4 = json.dumps(data2)
    #创建节点DEVEUI
    def post_api_create_deveui(hostname):
            url = "http://" + hostname + f":8080/api/devices"
            response = requests.post(url,data=data3, headers=header)
            return response
    #设置APPKEY
    def post_api_create_appkey(hostname):
            url = "http://" + hostname + f":8080/api/devices/{dev}/keys"
            response = requests.post(url, data=data4, headers=header)
            return response
    post_api_create_deveui(device_ip)
    time.sleep(3)
    post_api_create_appkey(device_ip)
    print(f"注册第{count}台成功,DEVEUI={dev}")
f.close()
