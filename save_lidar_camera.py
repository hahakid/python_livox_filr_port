#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import open3d as o3d
import openpylivox as opl
import time
import sys
import PySpin
import os
import cv2
import numpy as np
import glob
import utils
def acquire_images(cam, nodemap, img_path):
    # print('*** IMAGE ACQUISITION ***\n')
    try:
        # In order to access the node entries, they have to be casted to a pointer type (CEnumerationPtr here)
        node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode('AcquisitionMode'))
        if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
            print('Unable to set acquisition mode to continuous (enum retrieval). Aborting...')
            return False

        # Retrieve entry node from enumeration node
        node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
        if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(node_acquisition_mode_continuous):
            print('Unable to set acquisition mode to continuous (entry retrieval). Aborting...')
            return False

        # Retrieve integer value from entry node
        acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

        # Set integer value from entry node as new value of enumeration node
        node_acquisition_mode.SetIntValue(acquisition_mode_continuous)
        #print('Acquisition mode set to continuous...')

        cam.BeginAcquisition()
        #print('Acquiring images...')


        # Create ImageProcessor instance for post processing images
        processor = PySpin.ImageProcessor()
        processor.SetColorProcessing(PySpin.HQ_LINEAR)
        #fps = 0.0
        #btime = time.time()
        #count = 0
        try:
            image_result = cam.GetNextImage(1000)
            if image_result.IsIncomplete():
                print('Image incomplete with image status %d ...' % image_result.GetImageStatus())
            else:
                image_converted = processor.Convert(image_result, PySpin.PixelFormat_RGB8)
                img = image_converted.GetNDArray()
                image_result.Release()
                w, h, c = img.shape
                # print(img.shape)
                img = cv2.resize(img, (int(h/2), int(w/2)))  # resize to 1024*1024, the pixel is about 4 time of 1s point cloud (240, 000)
                img = np.rot90(img)  # rotation 90 degree, only need with livox holder
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                # print(img.shape)
                cv2.imwrite(img_path, img)

        except PySpin.SpinnakerException as ex:
            print('Error: %s' % ex)
            return False
        cam.EndAcquisition()

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False
    return img

def getMultiFrame(save_path, frames, lidar_slot=1):
    # lidar preparing
    sensor = opl.openpylivox(False)  # True=print message
    sensor.discover("192.168.1.51")  # connect based on fixed ip, 1.51=avia
    connected = sensor.auto_connect()  # build connect, sIP dIP data_port config_port
    # ['192.168.1.3', ['192.168.1.51'], 端口会一直变化 ['57273'], ['57274'], ['57275']]
    #connected = sensor.connect("192.168.1.3", "192.168.1.51", 57273, 57274, 57275)

    # camera preparing
    #result = True
    cam_system = PySpin.System.GetInstance()
    cam_list = cam_system.GetCameras()
    num_cameras = cam_list.GetSize()
    # handle
    if num_cameras == 0:
        cam_list.Clear()
        cam_system.ReleaseInstance()
        print('Not enough cameras!')
    else:
        # camera fps > lidar fps
        for cam_idx, cam in enumerate(cam_list):
            # nodemap_tldevice = cam.GetTLDeviceNodeMap()
            # Initialize camera
            cam.Init()
            # Retrieve GenICam nodemap
            nodemap = cam.GetNodeMap()
            # Acquire images


            # Deinitialize camera

            # connect lidar
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

                for i in range(frames):
                    sensor.dataStart_RT_B()
                    #print(i)
                    #sensor.lidarSpinUp()  # 开启
                    filePathAndName = os.path.join(save_path, str(i).zfill(6) + ".bin") #'pcds/frame.csv'
                    #secsToWait = 0
                    #duration = slot
                    sensor.saveDataToFile(filePathAndName, 0, lidar_slot)  # core fun
                    sensor.doneCapturing()
                    sensor.lidarStandBy()
                    #'''
                    while True:
                        #geting image and save
                        acquire_images(cam, nodemap, filePathAndName.replace(".bin", ".jpg"))
                        if sensor.doneCapturing():
                            #data capture is complete
                            break
                    #'''

                    sensor.lidarSpinUp()

            else:
                print("\n***** Could not connect to a Livox sensor *****\n")
            # release lidar
            sensor.lidarStandBy()
            sensor.dataStop()
            sensor.disconnect()
            # release cam
            cam.DeInit()
            # decode point cloud
            opl.convertBin2PCD(save_path)

    # release camera
        del cam
    cam_list.Clear()
    cam_system.ReleaseInstance()


def show_collected(path):

    imgs = glob.glob(path+"/*.jpg")
    pcds = glob.glob(path+"/*.pcd")
    if len(imgs) == len(pcds):
        for img in imgs:
            im = cv2.imread(img)
            cv2.imshow("Img window", im)
            pcd = o3d.io.read_point_cloud(img.replace('.jpg', '.pcd'))
            utils.pc_show([pcd])

            k = cv2.waitKey()
            if k == 27:
                break
        cv2.destroyAllWindows()


if __name__ == '__main__':
    save_path = './pcds'
    getMultiFrame(save_path, frames=5, lidar_slot=0.5)
    show_collected(save_path)
