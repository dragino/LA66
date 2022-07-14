#使用API通过requests库用POST请求在chripstack创建节点
import requests
import json
import time
f =open("Keys_WTS10_20220511_EU868.txt","r")
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
    auth_token = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhcGlfa2V5X2lkIjoiOGJmNjVmNzctMDkzYi00NTE3LWIzNTktMGI0NWVkNjM3M2Y5IiwiYXVkIjoiYXMiLCJpc3MiOiJhcyIsIm5iZiI6MTY1NDgzMDM5Nywic3ViIjoiYXBpX2tleSJ9.LPDOVb9AVMTR_AqOlqe_UwFtQSPsIOO9TSeJBGu4wjQ'    #POST headers
    header = {'Authorization': 'Bearer ' + auth_token}
    #创建节点,chripstack与TTN不同先需要通过DEVEUI创建节点后，再给节点设置APPKEY
    data1 = {
      "device": {
        "applicationID": "5",
        "description": "test",
        "devEUI": dev,
        "deviceProfileID": "06a341ed-3f2f-460e-97c1-02caf06c01c7",
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
        "appKey": "11111111111111111111111111111111",
        "devEUI": dev,
        "genAppKey": "11111111111111111111111111111111",
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
    post_api_create_deveui('101.200.134.147')
    time.sleep(3)
    post_api_create_appkey('101.200.134.147')
    print(f"注册第{count}台成功,DEVEUI={dev}")
f.close()
