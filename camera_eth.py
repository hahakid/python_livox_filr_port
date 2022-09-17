import PySpin
import os
import cv2
import numpy as np
import time

'''
Camera IP= 192.168.1.50
set computer to 192.168.1.X
auto set to continues acquisition mode
sampling at <10Hz
Quit: 1. press ESC at opencv window, 2. press Enter at terminal.

'''

def acquire_images(cam, nodemap, nodemap_tldevice, saveflag):
    print('*** IMAGE ACQUISITION ***\n')
    try:
        result = True

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
        print('Acquisition mode set to continuous...')

        cam.BeginAcquisition()
        print('Acquiring images...')

        device_serial_number = ''
        node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
        if PySpin.IsAvailable(node_device_serial_number) and PySpin.IsReadable(node_device_serial_number):
            device_serial_number = node_device_serial_number.GetValue()
            print('Device serial number retrieved as %s...' % device_serial_number)

        # Create ImageProcessor instance for post processing images
        processor = PySpin.ImageProcessor()
        processor.SetColorProcessing(PySpin.HQ_LINEAR)
        fps = 0.0
        btime = time.time()
        count = 0
        while True:
            try:
                image_result = cam.GetNextImage(1000)
                if image_result.IsIncomplete():
                    print('Image incomplete with image status %d ...' % image_result.GetImageStatus())

                else:
                    # width = image_result.GetWidth()
                    # height = image_result.GetHeight()
                    # print('Grabbed Image %d, width = %d, height = %d' % (i, width, height))

                    image_converted = processor.Convert(image_result, PySpin.PixelFormat_RGB8)
                    img = image_converted.GetNDArray()
                    image_result.Release()
                    filename = os.path.join('./imgs', '%s.jpg' % str(count).zfill(6))
                    # gtime = time.strftime('%Y/%m/%d %H:%M:%S'), ':%d' % (time.time() % 1.0*1000)
                    # filename = os.path.join('./imgs', gtime+".jpg")

                    w, h, c = img.shape
                    # print(img.shape)
                    img = np.rot90(img)  # rotation 90 degree, only need with livox holder
                    # print(img.shape)
                    if saveflag:
                        cv2.imwrite(filename, img)

                    img = cv2.resize(img, (int(h/2), int(w/2)))  # 显示调小一点
                    cost_time = time.time()-btime
                    fps = (count+1)/cost_time
                    img = cv2.putText(img, "fps:%f, frame:%s, pixel: %d * %d" % (fps, str(count).zfill(6), w, h),
                                      (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                    img = cv2.putText(img, "Press ESC to quit, and then press Enter to shutdown", (20, 80),
                                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

                    cv2.imshow("Img window", img)
                    k = cv2.waitKey(1)
                    if k == 27:
                        break
                    # image_converted.Save(filename)
                    count += 1

            except PySpin.SpinnakerException as ex:
                print('Error: %s' % ex)
                return False
        cam.EndAcquisition()
        cv2.destroyAllWindows()
    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False
    return result

def run_single_camera(cam, saveflag=True):
    """
    :param cam: Camera to run on.
    :type cam: CameraPtr
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    try:
        result = True

        nodemap_tldevice = cam.GetTLDeviceNodeMap()

        # Initialize camera
        cam.Init()

        # Retrieve GenICam nodemap
        nodemap = cam.GetNodeMap()

        # Acquire images
        result &= acquire_images(cam, nodemap, nodemap_tldevice, saveflag)

        # Deinitialize camera
        cam.DeInit()

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        result = False

    return result


if __name__ == '__main__':

    #print(time.strftime('%Y/%m/%d %H:%M:%S'), ':%d' % (time.time() % 1.0 * 1000))
    #time.sleep(0.01)
    #print(time.strftime('%Y/%m/%d %H:%M:%S'), ':%d' % (time.time() % 1.0 * 1000))
    #time.sleep(0.05)
    #print(time.strftime('%Y/%m/%d %H:%M:%S'), ':%d' % (time.time() % 1.0 * 1000))
    #'''
    # reg
    result = True
    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()
    num_cameras = cam_list.GetSize()
    # image save path check
    if not os.path.exists('./imgs'):
        os.makedir('./imgs')

    # handle
    if num_cameras == 0:
        cam_list.Clear()
        system.ReleaseInstance()
        print('Not enough cameras!')
        input('Done! Press Enter to exit...')

    for i, cam in enumerate(cam_list):
        result &= run_single_camera(cam, saveflag=False)

    #  release
    del cam
    cam_list.Clear()
    system.ReleaseInstance()

    #'''