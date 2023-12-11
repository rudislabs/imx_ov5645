#!/usr/bin/env python3
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType
from  sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge

if cv2.__version__ < "4.0.0":
    raise ImportError("Requires opencv >= 4.0, "
                      "but found {:s}".format(cv2.__version__))

class IMXOV5645Node(Node):

    def __init__(self):

        super().__init__("imx_ov5645_node")

        camera_image_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Camera image topic.')
        
        camera_info_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Camera info topic.')

        resolution_array_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description='Resolution in pixels [width, height]')

        framerate_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Frames per second as an integer.')

        device_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Device name ex: /dev/video3')

        rotation_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Hardware image rotation enum: (0): none, (1): rotate-90, (2): rotate-180, (3): rotate-270, (4): horizontal-flip, (5): vertical-flip')

        self.declare_parameter("camera_topic", "/ov5645/image_raw", 
            camera_image_topic_descriptor)

        self.declare_parameter("camera_info_topic", "/ov5645/camera_info", 
            camera_info_topic_descriptor)

        self.declare_parameter("resolution", [320, 240], 
            resolution_array_descriptor)

        self.declare_parameter("framerate", 30,
            framerate_descriptor)

        self.declare_parameter("device", "/dev/video3", 
            device_descriptor)

        self.declare_parameter("rotation", 2, 
            rotation_descriptor)

        self.cameraImageTopic = self.get_parameter("camera_topic").value
        self.resolution = self.get_parameter("resolution").value
        self.framerate = int(self.get_parameter("framerate").value)
        self.device = self.get_parameter("device").value
        self.rotation = self.get_parameter("rotation").value
        self.cameraInfoTopic = self.get_parameter("camera_info_topic").value


        #setup CvBridge
        self.bridge = CvBridge()

        self.ImagePub = self.create_publisher(Image,
            '{:s}'.format(self.cameraImageTopic), 0)
        self.ImageInfoPub = self.create_publisher(CameraInfo,
            '{:s}'.format(self.cameraInfoTopic), 0)

        videoCaptureString = 'v4l2src device={:s} ! video/x-raw,format=BGRx,framerate={:d}/1,width={:d},height={:d} ! imxvideoconvert_g2d rotation={:d} ! video/x-raw,format=BGRx,framerate={:d}/1,width={:d},height={:d} ! videoconvert ! video/x-raw,format=BGR ! appsink'.format(
            self.device, int(self.framerate), int(self.resolution[0]), int(self.resolution[1]), int(self.rotation), int(self.framerate), int(self.resolution[0]), int(self.resolution[1]))
        self.get_logger().debug('VideoCapture: {:s}'.format(videoCaptureString))
        
        self.videoCapture = cv2.VideoCapture(videoCaptureString, cv2.CAP_GSTREAMER)

        self.runCamera()

    def runCamera(self):
        msgInf = CameraInfo();
        msgInf.header.frame_id = "camera_link_optical"
        msgInf.height = self.resolution[1]
        msgInf.width = self.resolution[0]
        msgInf.distortion_model = "plumb_bob"
        msgInf.k = [381.5, 0.0, 160.0, 0.0, 381.5, 120.0, 0.0, 0.0, 1.0]
        msgInf.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msgInf.p = [381.5, 0.0, 160.0, 0.0, 0.0, 381.5, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]

        while(self.videoCapture.isOpened()):
            ret, frame = self.videoCapture.read()
            if ret == True:
                msgImg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                msgImg.header.stamp = self.get_clock().now().to_msg()
                msgImg.header.frame_id = msgInf.header.frame_id
                msgInf.header.stamp = msgImg.header.stamp
                self.ImagePub.publish(msgImg)
                self.ImageInfoPub.publish(msgInf)
            else:
                self.get_logger().warning("Video Stream Disconnected.")
                break

        if not self.videoCapture.isOpened():
            self.get_logger().error('Video Capture Device {:s} is not open, check settings and connections.'.format(self.device))

        return


def main(args=None):
    rclpy.init(args=args)
    node = IMXOV5645Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
