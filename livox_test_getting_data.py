#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import open3d as o3d
import openpylivox as opl
import time
import sys

#dIP = "192.168.1.51"
'''
1. 创建实体 sensor，发现设备，建立连接
2. 传感器配置
    2.1 雷达上电
    2.2 相关配置（livox viewer:device settings相关）： 外参、echo type, coordinate type, IMU, scan mode, high sensitivity,ip, etc.
    2.3 dataStart_RT_B()
        2.3.1.1 _dataCaptureThread()  # core fun
        2.3.1.2 self._cmdSocket.sendto() # 发请求
        2.3.1.3 self._cmdSocket.recvfrom(16)  # return binData, addr 
        2.3.1.4 self._parseResp(binData)  # return _, ack, cmd_set, cmd_id, ret_code_bin 
        2.3.1.5 int.from_bytes(ret_code_bin[0], byteorder='little')
        2.3.1.6 self._captureStream.stop()
    2.4 saveDataToFile()
    2.5 doneCapturing()        
    2.6 dataStop(), lidarStandby(), disconnect()
3. opl.convertBin2CSV() 完成数据文件转化
'''


def getsingleframe():

    # create a new lidar object
    sensor = opl.openpylivox(False)  # True=print message
    sensor.discover("192.168.1.51")  # connect based on fixed ip, 1.51=avia
    # sensor.discover("192.168.1.157")  # connect based on fixed ip, 1.157=horizon
    #sensor.showMessages(True)  # close sensor message
    connected = sensor.auto_connect()  # build connect, sIP dIP data_port config_port
    # ['192.168.1.3', ['192.168.1.51'], 端口会一直变化 ['57273'], ['57274'], ['57275']]
    #connected = sensor.connect("192.168.1.3", "192.168.1.51", 57273, 57274, 57275)
    if connected:
        # connParams = sensor.connectionParameters()
        # print(connParams)
        sensor.lidarSpinUp()  # 上电
        #firmware = sensor.firmware()
        #serial = sensor.serialNumber()
        #sensor.showMessages(True)
        sensor.readExtrinsic()  # default=[0,0,0, 0,0,0]
        # sensor.setExtrinsicToZero()  # extr-to-zero
        sensor.setLidarReturnMode(0)  # 0-single first 1-single strongest 2-dual return, none triple return currently
        sensor.setIMUdataPush(True)  # activate IMU, only: hrozion, avia, tele-15
        sensor.setRainFogSuppression(False)  # do not need in indoor environment

        #sensor.lidarSpinUp()  # 开启
        sensor.dataStart_RT_B()  # prepare data stream

        filePathAndName = 'pcds/frame.bin' #'pcds/frame.csv'
        secsToWait = 0.1
        duration = 0.1
        sensor.saveDataToFile(filePathAndName, secsToWait, duration)  # core fun

        while True:
        #doing some very important stuff here

            if sensor.doneCapturing():
                #data capture is complete
                break
        sensor.dataStop()
        sensor.lidarStandBy()
        # sensor.lidarSpinDown()  # 关闭


    else:
        print("\n***** Could not connect to a Livox sensor *****\n")

    sensor.disconnect()
    # opl.convertBin2CSV(filePathAndName, deleteBin=False)

def getMultiFrame():

    # create a new lidar object
    sensor = opl.openpylivox(False)  # True=print message
    sensor.discover("192.168.1.51")  # connect based on fixed ip, 1.51=avia
    # sensor.discover("192.168.1.157")  # connect based on fixed ip, 1.157=horizon
    #sensor.showMessages(True)  # close sensor message
    connected = sensor.auto_connect()  # build connect, sIP dIP data_port config_port
    # ['192.168.1.3', ['192.168.1.51'], 端口会一直变化 ['57273'], ['57274'], ['57275']]
    #connected = sensor.connect("192.168.1.3", "192.168.1.51", 57273, 57274, 57275)
    if connected:
        # connParams = sensor.connectionParameters()
        # print(connParams)
        sensor.lidarSpinUp()  # 上电
        sensor.readExtrinsic()  # default=[0,0,0, 0,0,0]
        # sensor.setExtrinsicToZero()  # extr-to-zero
        sensor.setLidarReturnMode(0)  # 0-single first 1-single strongest 2-dual return, none triple return currently
        sensor.setIMUdataPush(True)  # activate IMU, only: hrozion, avia, tele-15
        sensor.setRainFogSuppression(False)  # do not need in indoor environment
          # prepare data stream

        for i in range(5):

            sensor.dataStart_RT_B()
            #print(i)
            #sensor.lidarSpinUp()  # 开启
            filePathAndName = 'pcds/frame_%d.bin' % i #'pcds/frame.csv'
            secsToWait = 0
            duration = 1
            sensor.saveDataToFile(filePathAndName, secsToWait, duration)  # core fun
            sensor.doneCapturing()
            sensor.lidarStandBy()
            sensor.lidarSpinUp()
            '''
            while True:
                #doing some very important stuff here

                if sensor.doneCapturing():
                    #data capture is complete
                    break
            '''
            # sensor.lidarStandBy()
            # sensor.lidarSpinDown()  # 关闭

    else:
        print("\n***** Could not connect to a Livox sensor *****\n")


    sensor.lidarStandBy()
    sensor.dataStop()
    sensor.disconnect()
    opl.convertBin2PCD('pcds')
    # opl.convertBin2CSV(filePathAndName, deleteBin=False)

def test():
    sensor = opl.openpylivox(True)
    sensor.auto_connect()
    sensor.dataStart_RT_B()
    sensor.saveDataToFile("points.csv", 0, 0.1)   #collect exactly 10.0 seconds of data
    while True:
        if sensor.doneCapturing():
            break
    sensor.disconnect()
    opl.convertBin2LAS("points.csv", deleteBin=False)

def transformFile():
    filePathAndName = './pcds'
    #opl.convertBin2CSV(filePathAndName, deleteBin=False)
    opl.convertBin2PCD(filePathAndName)



if __name__ == '__main__':
    # transformFile()
    #test()
    # getsingleframe()
    getMultiFrame()
    # opl.convertBin2LAS('./1.bin', deleteBin=True)