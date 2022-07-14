import requests

auth_token = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhcGlfa2V5X2lkIjoiOGJmNjVmNzctMDkzYi00NTE3LWIzNTktMGI0NWVkNjM3M2Y5IiwiYXVkIjoiYXMiLCJpc3MiOiJhcyIsIm5iZiI6MTY1NDgzMDM5Nywic3ViIjoiYXBpX2tleSJ9.LPDOVb9AVMTR_AqOlqe_UwFtQSPsIOO9TSeJBGu4wjQ'
header = {'Authorization': 'Bearer ' + auth_token}
#查看数据需要将响应结果编码为JSON格式或者txt
def getdevices(ip):
    url = "http://" + ip + f":8080/api/devices?limit=10&applicationID=1"
    response = requests.get(url, headers=header)
    data = response.json()['result']
    for i in data:
        print(i['name'])
    #return data
print(getdevices('101.200.134.147'))
devtestid = '0111011111111111'
def deletedevices(ip):
    url = "http://" + ip + f":8080/api/devices/{devtestid}"
    response = requests.delete(url, headers=header)
    print(f"删除节点:DEVEUI={devtestid}")
    return response
#deletedevices('101.200.134.147')