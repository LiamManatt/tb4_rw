from pathlib import Path
import cv2
import depthai as dai
import numpy as np
import json
import math

import math
import numpy as np
import depthai as dai
from ament_index_python.packages import get_package_share_directory

class HostSpatialsCalc:
    # We need device object to get calibration data
    def __init__(self, device):
        self.calibData = device.readCalibration()

        # Values
        self.DELTA = 5
        self.THRESH_LOW = 200 # 20cm
        self.THRESH_HIGH = 30000 # 30m

    def setLowerThreshold(self, threshold_low):
        self.THRESH_LOW = threshold_low
    def setUpperThreshold(self, threshold_low):
        self.THRESH_HIGH = threshold_low
    def setDeltaRoi(self, delta):
        self.DELTA = delta

    def _check_input(self, roi, frame): # Check if input is ROI or point. If point, convert to ROI
        if len(roi) == 4: return roi
        if len(roi) != 2: raise ValueError("You have to pass either ROI (4 values) or point (2 values)!")
        # Limit the point so ROI won't be outside the frame
        self.DELTA = 5 # Take 10x10 depth pixels around point for depth averaging
        x = min(max(roi[0], self.DELTA), frame.shape[1] - self.DELTA)
        y = min(max(roi[1], self.DELTA), frame.shape[0] - self.DELTA)
        return (x-self.DELTA,y-self.DELTA,x+self.DELTA,y+self.DELTA)

    def _calc_angle(self, frame, offset, HFOV):
        return math.atan(math.tan(HFOV / 2.0) * offset / (frame.shape[1] / 2.0))

    # roi has to be list of ints
    def calc_spatials(self, depthData, roi, averaging_method=np.mean):

        depthFrame = depthData.getFrame()

        roi = self._check_input(roi, depthFrame) # If point was passed, convert it to ROI
        xmin, ymin, xmax, ymax = roi

        print(roi)

        # Calculate the average depth in the ROI.
        depthROI = depthFrame[ymin:ymax, xmin:xmax]
        inRange = (self.THRESH_LOW <= depthROI) & (depthROI <= self.THRESH_HIGH)

        # Required information for calculating spatial coordinates on the host
        HFOV = np.deg2rad(self.calibData.getFov(dai.CameraBoardSocket(depthData.getInstanceNum()), useSpec=False))


        averageDepth = averaging_method(depthROI[inRange])

        centroid = { # Get centroid of the ROI
            'x': int((xmax + xmin) / 2),
            'y': int((ymax + ymin) / 2)
        }

        midW = int(depthFrame.shape[1] / 2) # middle of the depth img width
        midH = int(depthFrame.shape[0] / 2) # middle of the depth img height
        bb_x_pos = centroid['x'] - midW
        bb_y_pos = centroid['y'] - midH

        angle_x = self._calc_angle(depthFrame, bb_x_pos, HFOV)
        angle_y = self._calc_angle(depthFrame, bb_y_pos, HFOV)

        spatials = {
            'z': averageDepth,
            'x': averageDepth * math.tan(angle_x),
            'y': -averageDepth * math.tan(angle_y)
        }
        return spatials, centroid



package_share_directory = get_package_share_directory('tb4_rw') + '/data'


# parse config
configPath = Path(package_share_directory +'/last.json')
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
nnPath = package_share_directory + "/last_openvino_2022.1_5shave.blob"

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

height = 640
width  = 640

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    #program crashes without all these defined, though only detection is called after this
    previewQueue = device.getOutputQueue(name="rgb", maxSize=20, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=20, blocking=False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=20, blocking=False)
    networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=20, blocking=False)


    while True:
        hostSpatials = HostSpatialsCalc(device)
        delta = 5
        hostSpatials.setDeltaRoi(delta)
        inDet = detectionNNQueue.get()
        depthData = depthQueue.get()



        detections = inDet.detections
        
    

        # if not detections:
        #     with open(working_file, 'a') as file:
        #         file.write("[None, None, None]\r")
        for detection in detections:
            roiData = detection.boundingBoxMapping
            roi = roiData.roi
            roi = roi.denormalize(640,640)
            topLeft = roi.topLeft()
            bottomRight = roi.bottomRight()
            xmin = int(topLeft.x)
            ymin = int(topLeft.y)
            xmax = int(bottomRight.x)
            ymax = int(bottomRight.y)
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)
            spatials, centroid = hostSpatials.calc_spatials(depthData, (x1,y2))
            spatials1, centroid1 = hostSpatials.calc_spatials(depthData, (x2,y2))
            print(spatials,spatials1)

        if cv2.waitKey(1) == ord('q'):
            break
