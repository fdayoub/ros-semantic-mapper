#!/usr/bin/env python
'''
Copyright (C) 2016, by 
Feras Dayoub (feras.dayoub@gmail.com) 

This is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
 
This software package is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Leser General Public License.
If not, see <http://www.gnu.org/licenses/>.
'''
import roslib; roslib.load_manifest('semantic_label_publisher') 
import numpy as np
import rospy
import sys
import cv2
import cPickle
import gzip
import caffe
from sklearn.externals import joblib
from sklearn import svm
import os.path
import time
from read_cat import cats
from sensor_msgs.msg import Image , LaserScan

from cv_bridge import CvBridge, CvBridgeError

from semantic_label_publisher.msg import SemLabel
class SemanticLabel(object):
  def __init__(self,lname,lid,lcolor):
    self.label_name  = lname
    self.label_id    = int(lid)
    self.label_color = [int(lcolor[0]),int(lcolor[1]),int(lcolor[2])]

class SemLabelPub():
  def __init__(self,caffe_root,MODEL_FILE,PRETRAINED,MEAN_FILE,SVM_PICKLE_FILE,SUB_CAT_FILE):
    self.pub       = rospy.Publisher('semantic_label',SemLabel)
    self.image_pub = rospy.Publisher("sem_label_image",Image)

    self.image_topic = rospy.get_param('~image_topic','camera/rgb/image_raw_throttle')
    self.image_sub   = rospy.Subscriber(self.image_topic,Image,self.image_callback)

    self.bridge = CvBridge()

    self.net         = caffe.Classifier(MODEL_FILE, PRETRAINED,caffe.TEST)
    self.transformer = caffe.io.Transformer({'data': (1,3,227,227)})

    self.transformer.set_transpose('data', (2,0,1))
    self.net.blobs['data'].reshape(1,3,227,227)

    db_mean = np.load(MEAN_FILE)
    self.transformer.set_mean('data', db_mean.mean(1).mean(1))

    caffe.set_mode_gpu()

    self.msg = SemLabel()

    with open(SUB_CAT_FILE, 'r') as f:
      txt_data = f.readlines()

    self.labels = list()
    for l in txt_data:
      if l[0] != '#':
        label_name    = l.split(" ")[0]
        label_id      = l.split(" ")[1] 
        label_color   = l.split(" ")[2].split(",")
        self.labels.append(SemanticLabel(label_name,label_id,label_color))
 
    
    self.font_size = 3
    self.font_thickness = 5
      
  def image_callback(self,data):
    print "new image received"
    try:
      cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv_img = cv2.resize(cv_img,(227,227))
      print 'image received'
    except CvBridgeError, e:
      print e

    im_input = self.transformer.preprocess('data',cv_img) 
    im_input = im_input[np.newaxis] 
    self.net.blobs['data'].reshape(*im_input.shape)
    self.net.blobs['data'].data[...] = im_input
    self.net.forward()
    
    if 'fc7' in self.net.blobs.keys():
      feature  =  self.net.blobs['fc7'].data
      prob     =  self.net.blobs['prob'].data
      result          = list()
      self.all_lables = list() 
      for l in self.labels:
          idx = l.label_id
          self.all_lables.append(l.label_name)
          result.append(prob[0,idx])
      result = np.array(result,np.dtype(float))
      result = result / np.sum(result)
    
    self.msg.header.stamp = data.header.stamp
    self.msg.header.frame_id = 'base_link'
    class_idx = np.argmax(result)    
    class_name = self.all_lables[class_idx]
    print class_name
    
    self.msg.r = [k.label_color[0] for k in self.labels]
    self.msg.g = [k.label_color[1] for k in self.labels] 
    self.msg.b = [k.label_color[2] for k in self.labels] 
    self.msg.prob = result 
    self.msg.lvl = class_idx

    self.pub.publish(self.msg)
    text_x = 10
    text_y = 100
    cv_img = cv2.resize(cv_img,(1280,960))
    font = cv2.FONT_HERSHEY_PLAIN
    for c in range(len(result)):
      cv2.rectangle(cv_img,(text_x+0,text_y),(text_x+0+int(300*float(result[c])),text_y - 20), (255,0,0),20)
      if c == class_idx:
        cv2.putText(cv_img,self.all_lables[c] + ' ' ,(text_x,text_y), font, self.font_size,(0,255,0),self.font_thickness)        
      else:
        cv2.putText(cv_img,self.all_lables[c] + ' ' ,(text_x,text_y), font, self.font_size,(0,0,255),self.font_thickness)        
      text_y += 50
      
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))
    except CvBridgeError, e:
      print e

    

def main(args):
  rospy.init_node('sem_label_pub')
  caffe_root      = rospy.get_param('~caffe_root', '~/caffe/')
  MODEL_FILE      = rospy.get_param('~MODEL_FILE_PATH', '~/deploy.prototxt')
  PRETRAINED      = rospy.get_param('~PRETRAINED_PATH', '~/model.caffemodel')
  MEAN_FILE       = rospy.get_param('~MEAN_FILE_PATH', '~/mean.npy')
  SVM_PICKLE_FILE = rospy.get_param('~SVM_PICKLE_FILE_PATH', '~/clf.pkl')
  SUB_CAT_FILE    = rospy.get_param('~SUB_CAT_FILE','~/sub_cats.txt')
  try:
    ne = SemLabelPub(caffe_root,MODEL_FILE,PRETRAINED,MEAN_FILE,SVM_PICKLE_FILE,SUB_CAT_FILE) 
  except rospy.ROSInterruptException: pass
  rospy.spin()
  
if __name__ == '__main__':
  main(sys.argv)
    
  

