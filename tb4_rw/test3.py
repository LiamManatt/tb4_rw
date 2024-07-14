from pathlib import Path
import cv2
import depthai as dai
import numpy as np
import time
import json



working_file = '/home/ubuntu/ros2_ws/src/tb4_rw/tb4_rw/data2.txt'

# parse config
configPath = Path('last.json')
if not configPath.exists():
    raise ValueError("Path {} does not exist!".format(configPath))

with configPath.open() as f:
    config = json.load(f)
nnConfig = config.get("nn_config", {})

# parse input shape
if "input_size" in nnConfig:
    W, H = tuple(map(int, nnConfig.get("input_size").split('x')))

# extract metadata
metadata = nnConfig.get("NN_specific_metadata", {})
classes = metadata.get("classes", {})
coordinates = metadata.get("coordinates", {})
anchors = metadata.get("anchors", {})
anchorMasks = metadata.get("anchor_masks", {})
iouThreshold = metadata.get("iou_threshold", {})
confidenceThreshold = metadata.get("confidence_threshold", {})


# parse labels
nnMappings = config.get("mappings", {})
labels = nnMappings.get("labels", {})
labelMap = labels
# get model path
nnPath = "last_openvino_2022.1_5shave.blob"

syncNN = True


# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
nnNetworkOut = pipeline.create(dai.node.XLinkOut)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutDepth.setStreamName("depth")
nnNetworkOut.setStreamName("nnNetwork")

# Properties
camRgb.setPreviewSize(640, 640)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# setting node configs
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Align depth map to the perspective of RGB camera, on which inference is done
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
stereo.setSubpixel(True)

# Network specific settings
spatialDetectionNetwork.setConfidenceThreshold(confidenceThreshold)
spatialDetectionNetwork.setNumClasses(classes)
spatialDetectionNetwork.setCoordinateSize(coordinates)
spatialDetectionNetwork.setAnchors(anchors)
spatialDetectionNetwork.setAnchorMasks(anchorMasks)
spatialDetectionNetwork.setIouThreshold(iouThreshold)
spatialDetectionNetwork.setBlobPath(nnPath)
spatialDetectionNetwork.setNumInferenceThreads(1)
spatialDetectionNetwork.input.setBlocking(False)


# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
if syncNN:
    spatialDetectionNetwork.passthrough.link(xoutRgb.input)
else:
    camRgb.preview.link(xoutRgb.input)

spatialDetectionNetwork.out.link(xoutNN.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
spatialDetectionNetwork.outNetwork.link(nnNetworkOut.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    #program crashes without all these defined, though only detection is called after this
    previewQueue = device.getOutputQueue(name="rgb", maxSize=20, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=20, blocking=False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=20, blocking=False)
    networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=20, blocking=False)


    while True:
        inDet = detectionNNQueue.get()


        detections = inDet.detections

        # If the frame is available, draw bounding boxes on it and show the frame
        if not detections:
            with open(working_file, 'a') as file:
                file.write("[None, None, None]\r")
        for detection in detections:
            with open(working_file, 'a') as file:
               file.write(f"[{int(detection.spatialCoordinates.x)/1000}, {int(detection.spatialCoordinates.y)/1000}, {int(detection.spatialCoordinates.z)/1000}]\r")

        if cv2.waitKey(1) == ord('q'):
            break
