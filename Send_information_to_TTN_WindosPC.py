import serial
import psutil
import wmi
import time
ser = serial.Serial()
ser.port = 'com3'
ser.baudrate = 115200
ser.bytesize = 8
ser.parity = 'N'
ser.stopbits = 1
ser.timeout = None
ser.xonxoff = 0
ser.rtscts = 0
ser.open()
def redline1(str):
        str1 = f"AT+SENDB=01,02,{int(len(str)/2)},{str}\n"
        str1 = str1.encode(encoding="UTF-8")
        print(str1)
        ser.write(str1)
# cpu信息：获取cpu使用率
def get_cpu_info(): #单位%
    cpu_percent = psutil.cpu_percent(interval=1)
    cpu_info = cpu_percent
    #print(cpu_info)
    cpu_info = int(cpu_info*100)
    cpu_info = str(hex(cpu_info))[2:]
    if len(cpu_info)%2 != 0:
        cpu_info = '0'+ cpu_info
    else:
        cpu_info = cpu_info
    return cpu_info
# 内存信息：内存使用，使用率，剩余内存
def get_memory_info():
    w = wmi.WMI()
    virtual_memory = psutil.virtual_memory()
    used_memory = virtual_memory.used/1024/1024/1024
    free_memory = virtual_memory.free/1024/1024/1024
    memory_percent = virtual_memory.percent
    # print(used_memory)
    # print(free_memory)
    # print(memory_percent)
    # for memModule in w.Win32_PhysicalMemory():
    #     #totalMemSize=(int(memModule.Capacity))/1024**3#每个核的容量单位G
    #     #totalMemSize = str(hex(int(totalMemSize)))[2:]
    used_memory = str(hex(int(round(used_memory, 2)*100)))[2:]#使用内存单位G
    memory_percent = str(hex(int(round(memory_percent, 2)*100)))[2:]#内存使用率单位%
    free_memory = str(hex(int(round(free_memory, 2)*100)))[2:]#空闲内存单位G
    # if len(totalMemSize)%2 != 0:
    #     totalMemSize = '0'+ totalMemSize
    # else:
    #     totalMemSize = totalMemSize
    if len(used_memory)%2 != 0:
        used_memory = '0'+ used_memory
    else:
        used_memory = used_memory
    if len(memory_percent)%2 != 0:
        memory_percent = '0'+ memory_percent
    else:
        memory_percent = memory_percent
    if len(free_memory)%2 != 0:
        free_memory = '0'+ free_memory
    else:
        free_memory = free_memory
    memory_info = used_memory, memory_percent,free_memory
    return memory_info
while True:
    get_cpu_info()
    get_memory_info()
    redline1(get_cpu_info()+get_memory_info()[0]+get_memory_info()[1]+get_memory_info()[2])
    time.sleep(60)
