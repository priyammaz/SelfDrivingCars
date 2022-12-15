import cv2
import argparse
import numpy as np
import torch
import torchvision
from matplotlib import pyplot as plt
import time

import rospy
from sensor_msgs.msg import Image
from detection_outbound.msg import DetectMessage # TODO this needs to be created and instantiated

from utils import *
from darknet import Darknet

class DetectAndStop:
    def __init__(self):
        self.rate = rospy.Rate(0.25)
        self.pacmod_enable = False
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

        self.detector = Darknet('yolov3.cfg')
        self.detector.load_weights('yolov3.weights')

        self.detector.to(self.device)
        self.class_names = load_class_names('yolov3.txt')
        self.image = None
        self.zed_sub = rospy.Subscriber('/zed2/zed_node/stereo/image_rect_color', Image, self.zed_callback)
        self.image_size = (self.detector.width, self.detector.height)
        self.pub = rospy.Publisher('detect_publisher', DetectMessage)
        self.msg = DetectMessage()
        self.msg.detected_x = []
        self.msg.detected_y = []
        self.msg.detected_depth = []
        self.msg.latest_time = time.time()

    def zed_callback(self, msg):
        self.original_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self.depth_channel = self.original_image[:, :, -1]
        self.original_image = self.original_image[:, :, :3]
        self.image = cv2.resize(self.original_image, self.image_size).astype('float')
        if msg.height > 0 and not self.pacmod_enable:
            self.pacmod_enable = True
            # self.detector = YOLODetector(msg.width, msg.height)

    def start_pacmod(self):
        while not rospy.is_shutdown():
            if self.pacmod_enable:
                plt.close()
                boxes = detect_objects(self.detector, self.image, 0.4, 0.6, self.device)
                if len(boxes) > 0:
                    self.msg.detected_x = []
                    self.msg.detected_y = []
                    self.msg.detected_depth = []
                    boxes = torch.tensor(boxes)
                    # print(boxes.shape)
                    filt = boxes[:, -1] == 0
                    boxes = boxes[filt]
                    # plot_boxes(self.original_image, boxes, self.class_names, plot_labels=False)
                    # print(boxes.shape)
                    width = self.original_image.shape[1]
                    height = self.original_image.shape[0]
                    x1s = int(np.around((boxes[:, 0] - boxes[:, 2]/2.0) * width))
                    y1s = int(np.around((boxes[:, 1] - boxes[:, 3]/2.0) * height))
                    x2s = int(np.around((boxes[:, 0] + boxes[:, 2]/2.0) * width))
                    y2s = int(np.around((boxes[:, 1] + boxes[:, 3]/2.0) * height))

                    for x1, y1, x2, y2 in zip(x1s, y1s, x2s, y2s):
                        depth_slice = self.original_image[y1:y2, x1:x2]
                        self.msg.detected_x.append((x2 + x1) / 2)
                        self.msg.detected_y.append((y2 + y1) / 2)
                        self.msg.detected_depth.append(np.mean(depth_slice))
                else:
                    self.msg.detected_x = []
                    self.msg.detected_y = []
                    self.msg.detected_depth = []


                self.msg.latest_time = time.time()
                rospy.loginfo(self.msg)
                self.pub.publish(self.msg)
                # res = self.detector.predict_person_bounding_box(self.image)
                # print(res)
            self.rate.sleep()



class YOLODetector:
    def __init__(self,
                image_width,
                image_height,
                path_to_config="yolov3.cfg",
                path_to_weights="yolov3.weights",
                path_to_class_labels="yolov3.txt",
                conf_threshold=0.5,
                nms_threshold=0.4):
        
        ### MODEL DEPENDENCIES ###
        self.width = image_width
        self.height = image_height
        self.scale = 0.00392
        with open(path_to_class_labels, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]    
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        
        ### MODEL IMPORTS ###
        self.net = cv2.dnn.readNet(path_to_weights, path_to_config)
        self.output_layers = [self.net.getLayerNames()[i - 1] for i in self.net.getUnconnectedOutLayers()]
#         self.output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    
    def predict_person_bounding_box(self, image, person_only=True, draw=False):
        blob = cv2.dnn.blobFromImage(image, self.scale, (416,416), (0,0,0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)
        class_ids, confidences, boxes = self.grab_bounding_boxes(outs)
        
        if person_only:
            person_box_idx = [i for i in list(range(len(class_ids))) if class_ids[i] == 0]
            class_ids = [class_ids[i] for i in person_box_idx]
            boxes = [boxes[i] for i in person_box_idx]
            confidences = [confidences[i] for i in person_box_idx]
         
        selected_bounds, selected_confidences, selected_class_ids = self.nonmax_supression(boxes, confidences, image,class_ids)
        
        if draw:
            for box, confidence, class_id in zip(selected_bounds, selected_confidences, selected_class_ids):
                x, y, w, h = box
                image = self.draw_prediction(image, class_id, confidence, round(x), round(y), round(x+w), round(y+h))
                
            cv2.imshow("object detection", image)
            cv2.waitKey()
            
            return image, selected_bounds
        
        return selected_bounds
            
        
    def grab_bounding_boxes(self, outs):
        class_ids = []
        confidences = []
        boxes = []
        
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > self.conf_threshold:
                    center_x = int(detection[0] * self.width)
                    center_y = int(detection[1] * self.height)
                    w = int(detection[2] * self.width)
                    h = int(detection[3] * self.height)
                    x = center_x - w / 2
                    y = center_y - h / 2
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([x, y, w, h])
        return class_ids, confidences, boxes
    
    def nonmax_supression(self, boxes, confidences, image, class_ids):
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)
        selected_bounds = []
        selected_confidences = []
        selected_class_ids = []
        for i in indexes:
            try:
                box = boxes[i]
                selected_bounds.append(box)
                selected_confidences.append(confidences[i])
                selected_class_ids.append(class_ids[i])
            except:
                i = i[0]
                box = boxes[i]
                selected_bounds.append(box)
                selected_confidences.append(confidences[i])
                selected_class_ids.append(class_ids[i])
        
        return selected_bounds, selected_confidences, selected_class_ids
        
    def draw_prediction(self, img, class_id, confidence, x, y, x_plus_w, y_plus_h):
        label = str(self.classes[class_id])
        color = self.colors[class_id]
        cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)
        cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return img
    

def pacmod_run():
    rospy.init_node('pacmod_control_node', anonymous=True)
    pacmod = DetectAndStop()

    try:
        pacmod.start_pacmod()

    except rospy.RosInterruptException:
        pass

if __name__ == '__main__':
    pacmod_run()                    
        
        
# if __name__ == "__main__":
#     image = cv2.imread("sample.jpg")
#     h,w,c = image.shape
#     yolo = YOLODetector(image_width=w, image_height=h)
#     yolo.predict_person_bounding_box(image, draw=True)
