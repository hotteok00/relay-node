IMAGE_RAW           = '/image_raw/compressed'
JOINT_STATES        = '/joint_states'
IMAGE_AND_JOINT     = '/image_and_joint'
INFERENCE           = "/inference"
ACTION_CHUNK        = "/action_chunk"
JOINT_COMMMAND      = '/joint_command'
MOVEIT_JOINT_STATES = "/dsr_moveit_controller/controller_state"
JOINT_TRAJECTORY    = "/dsr_moveit_controller/joint_trajectory"

namespace = '/dsr01'

max_queue_len = 10

# ============= realsense camera ============= #

from sensor_msgs.msg import CameraInfo, Image, CompressedImage, PointCloud2
from theora_image_transport.msg import Packet
from realsense2_camera_msgs.msg import Metadata, Extrinsics, RGBD

REALSENSE = "/camera/camera"
aligned_depth_to_color = {
    "/aligned_depth_to_color/camera_info": CameraInfo,
    "/aligned_depth_to_color/image_raw": Image,
    "/aligned_depth_to_color/image_raw/compressed": CompressedImage,
    "/aligned_depth_to_color/image_raw/compressedDepth": CompressedImage,
    "/aligned_depth_to_color/image_raw/theora": Packet
}
color = {
    "/color/camera_info": CameraInfo,
    "/color/image_raw": Image,
    "/color/image_raw/compressed": CompressedImage,
    "/color/image_raw/compressedDepth": CompressedImage,
    "/color/image_raw/theora": Packet,
    "/color/metadata": Metadata
}
depth = {
    "/depth/camera_info": CameraInfo,
    "/depth/color/points": PointCloud2,
    "/depth/image_rect_raw": Image,
    "/depth/image_rect_raw/compressed": CompressedImage,
    "/depth/image_rect_raw/compressedDepth": CompressedImage,
    "/depth/image_rect_raw/theora": Packet,
    "/depth/metadata": Metadata,
}
extrinsics={"/extrinsics/depth_to_color": Extrinsics}
rgbd={"/rgbd": RGBD}

REALSENSE_IMAGE=REALSENSE+"/color/image_raw/compressed"