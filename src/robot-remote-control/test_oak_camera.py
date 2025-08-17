#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

def main():
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    camRgb = pipeline.createColorCamera()
    xoutRgb = pipeline.createXLinkOut()

    xoutRgb.setStreamName("rgb")

    # Properties
    camRgb.setPreviewSize(300, 300)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    # Linking
    camRgb.preview.link(xoutRgb.input)

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        print('Connected cameras:', device.getConnectedCameras())
        
        # Get output queue
        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        while True:
            inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived
            
            # Retrieve frame
            frame = inRgb.getCvFrame()
            
            # Show the frame
            cv2.imshow("RGB", frame)

            # Exit on 'q' press
            if cv2.waitKey(1) == ord('q'):
                break

if __name__ == "__main__":
    main() 