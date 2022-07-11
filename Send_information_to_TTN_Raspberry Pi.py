import os
import serial
import time
ser = serial.Serial('/dev/ttyUSB1')
ser.baudrate = 9600
ser.bytesize = 8
ser.parity = 'N'
ser.stopbits = 1
ser.timeout = None
ser.xonxoff = 0
ser.rtscts = 0
#Return CPU temperature as a character string
def redline1(str):
    str1 = f"AT+SENDB=01,03,{int(len(str) / 2)},{str}\n"
    str1 = str1.encode(encoding="UTF-8")
    print(str1)
    ser.write(str1)
def getCPUtemperature():
    res = os.popen('vcgencmd measure_temp').readline()
    return (res.replace("temp=", "").replace("'C",""))
    # Return RAM information (unit=kb) in a list
    # Index 0: total RAM
    # Index 1: used RAM
    # Index 2: free RAM
def getRAMinfo():
    p = os.popen('free')
    i = 0
    while 1:
        i = i + 1
        line = p.readline()
        if i == 2:
            return (line.split()[1:4])
# Return % of CPU used by user as a character string
def getCPUuse():
    return (str(os.popen("top -n1 | awk '/Cpu\(s\):/ {print $2}'").readline().strip()))
while True:
    # CPU informatiom
    CPU_temp = int(float(getCPUtemperature().strip())*10) #CPU温度
    cpu_temp = str(hex(int(CPU_temp)))[2:]
    if len(cpu_temp)%2 !=0:
        cpu_temp = '0'+ cpu_temp
    CPU_usage = int(float(getCPUuse().strip())*10)#CPU利用率
    cpu_usage = str(hex(int(CPU_usage)))[2:]
    if len(cpu_usage)%2 != 0:
        cpu_usage = '0'+ cpu_usage
    if len(cpu_usage)%2 ==0 and len(cpu_usage)==2:
        cpu_usage = '00'+ cpu_usage
    # RAM information
    # Output is in kb, here I convert it in Mb for readability
    RAM_stats = getRAMinfo()#获取内存信息
    RAM_total = str(hex(int(round(int(RAM_stats[0]) / 1024, 1)*10)))[2:] #内存大小
    if len(RAM_total)%2 !=0:
        RAM_total = '0'+RAM_total
    RAM_used = str(hex(int(round(int(RAM_stats[1]) / 1024, 1)*10)))[2:] #已用内存
    if len(RAM_used)%2 != 0:
        RAM_used = '0'+RAM_used
    RAM_free = str(hex(int(round(int(RAM_stats[2]) / 1024, 1)*10)))[2:] # 剩余内存
    if len(RAM_free)%2 != 0:
        RAM_free = '0'+ RAM_free
    redline1(cpu_temp+cpu_usage+RAM_total+RAM_used+RAM_free)
    time.sleep(60)