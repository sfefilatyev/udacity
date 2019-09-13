from styx_msgs.msg import TrafficLight
import rospy
import tensorflow
import numpy as np
import time
import os
import errno
from PIL import Image

class TLClassifier(object):
    def __init__(self, tl_model_path, min_detect_score_thresh=0.5):
        # load classifier
        tl_model_file_path = os.getcwd() + tl_model_path           # os.getcwd() points to tl_detector.py dir where the code exec
        rospy.loginfo('>>> TLClassifier is using model at: %s, min TL detection threshold: %s', tl_model_file_path, str(min_detect_score_thresh))

        if not os.path.exists(tl_model_file_path) or not os.path.isfile(tl_model_file_path):
            rospy.logerror('Cannot find the model file configured in traffic light configs yaml.')
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), tl_model_file_path)

        # load NN graph
        self.detection_graph = self.load_graph(tl_model_file_path)
        self.min_detect_thresh = min_detect_score_thresh

        self.categoty_to_tl_map = {'1': TrafficLight.GREEN, '2': TrafficLight.RED, '3': TrafficLight.YELLOW, '4': TrafficLight.UNKNOWN}
        self.categoty_to_str_map = {'1': 'GREEN', '2': 'RED', '3': 'YELLOW', '4': 'UNKNOWN'}
        
        config = tensorflow.ConfigProto()
        config.gpu_options.allow_growth = True

        self.sess = tensorflow.Session(graph=self.detection_graph, config=config)
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detect_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detect_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detect_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        image_np = self.load_image_into_numpy_array(image)
        image_expanded = np.expand_dims(image_np, axis=0)

        t0 = time.time()
        (boxes, scores, classes, num) = self.sess.run(
            [self.detect_boxes, self.detect_scores, self.detect_classes, self.num_detections],
            feed_dict={self.image_tensor: image_expanded})
        t1 = time.time()

        traff_light = TrafficLight.UNKNOWN
        detect_score = 0
        tl_col_str = self.categoty_to_str_map['4']
        if scores is not None and len(scores[0]) > 0:
            detect_score = np.squeeze(scores)[0]
            if detect_score > self.min_detect_thresh:
                # A positive detection happened
                cat_idx_str = str(np.squeeze(classes).astype(np.int32)[0])
                tl_col_str = self.categoty_to_str_map[cat_idx_str]
                traff_light = self.categoty_to_tl_map[cat_idx_str]

        rospy.logdebug('----------------------')

        rospy.loginfo('TLClassifier detected %s (score %s) light after %d msec', tl_col_str, str(detect_score), (t1 - t0) * 1000)

        rospy.logdebug('TLClassifier SCORES %s', str(scores[0]))
        rospy.logdebug('TLClassifier CLASSES %s', str(classes[0]))
        rospy.logdebug('----------------------')

        return traff_light

    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tensorflow.Graph()
        with graph.as_default():
            od_graph_def = tensorflow.GraphDef()
            with tensorflow.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tensorflow.import_graph_def(od_graph_def, name='')
        return graph

    def load_image_into_numpy_array(self, image):
        image = Image.fromarray(image)
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)
