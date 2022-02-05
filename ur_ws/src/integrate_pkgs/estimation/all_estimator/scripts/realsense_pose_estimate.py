import sys
import os
from xml.etree.ElementTree import TreeBuilder
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/raugh_recognition/'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/raugh_recognition/pointnet_pose'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/semantic_segmentation'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/semantic_segmentation/pointnet_semantic'))
from semantic_segmentation import pointnet_semantic

from raugh_recognition.pointnet_pose.options.test_options import TestOptions_raugh_recognition
from semantic_segmentation.pointnet_semantic.options.test_options import TestOptions_semantic_segmentation
from semantic_segmentation import pointnet_semantic
from raugh_recognition import pointnet_pose
from raugh_recognition.pointnet_pose import pointnet_exec
from semantic_segmentation.pointnet_semantic import semantic_exec
from common_function.cloud_util import *
from std_msgs.msg import Header
import numpy as np
import time
from denso_srvs.srv import bounding, boundingResponse
from denso_msgs.msg import out_segmentation
