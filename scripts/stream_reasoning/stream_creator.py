#!/usr/bin/env python

import rospy
import rdflib
import cv2
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose
from locobot_simulation.msg import LogicalImage
from cv_bridge import CvBridge, CvBridgeError
import csv
import datetime
import os

class KnowledgeGraphBuilder:
    def __init__(self):
        # Initialize node
        rospy.init_node('knowledge_graph_builder_node', anonymous=True)

        # Set up subscribers
        rospy.Subscriber('/gazebo/locobot/camera/logical_camera_image', LogicalImage, self.logical_camera_callback)
        rospy.Subscriber('/locobot/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/locobot/camera/color/image_raw', Image, self.camera_callback)

        self.dir = os.getcwd()
        # Set up cv_bridge for image conversion
        self.bridge = CvBridge()
        # Set up RDF graph and namespaces
        self.graph = rdflib.Graph()
        self.owl = rdflib.Namespace("http://www.w3.org/2002/07/owl#")
        self.rdf = rdflib.Namespace("http://www.w3.org/1999/02/22-rdf-syntax-ns#")
        self.rdfs = rdflib.Namespace("http://www.w3.org/2000/01/rdf-schema#")
        self.owl2rl = rdflib.Namespace("http://www.w3.org/2002/03owlt/rl#")
        self.sosa = rdflib.Namespace("http://www.w3.org/ns/sosa/#") 
        self.dul = rdflib.Namespace("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#")
        self.soma = rdflib.Namespace("http://www.ease-crc.org/ont/SOMA.owl#")
        self.time = rdflib.Namespace("https://www.w3.org/TR/2022/CRD-owl-time-20221115#")

        # Set up some constants
        self.panValue = 0
        self.pan = rdflib.URIRef('http://example.com/locobot/pan#')
        self.seenAt = rdflib.URIRef('http://example.com/locobot/seenAt#')
        self.hasJointValue = rdflib.URIRef('http://example.com/locobot/hasJointValue#')
        self.hasSize = rdflib.URIRef('http://example.com/locobot/hasSize#')
        self.counter = 0
        self.timer = 0
        self.done = False
        self.image = None

        self.recordedGraphs = []

        # Set the publishing rate
        self.rate = rospy.Rate(100) # 1 Hz

    def build_stream(self):
        while not rospy.is_shutdown():
            #rospy.loginfo(self.graph)
            # Sleep to maintain the publishing rate
            self.rate.sleep()

    def logical_camera_callback(self, data):
        # Extract information from LogicalCameraImage
        self.graph = rdflib.Graph()
        # Get current time and date
        ts = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")
        obs = rdflib.URIRef('http://example.com/locobot/Observation#' + ts)
        rospy.loginfo(self.dir)
        # Create filename with current time and date
        timestamp = rdflib.Literal(ts)
        img_name = 'streams/images/image' + str(timestamp) + '.jpg'
        imgNode = rdflib.Literal(img_name)
        self.graph.add((obs, self.time.hasTime, timestamp))
        pan_joint_value = rdflib.Literal(self.panValue)
        self.graph.add((imgNode, self.time.hasTime, timestamp))
        self.graph.add((obs, self.rdf.type, self.sosa.Observation))
        self.graph.add((self.pan, self.hasJointValue, pan_joint_value))
        self.graph.add((self.pan, self.time.hasTime, pan_joint_value))
        self.graph.add((self.pan, self.rdf.type, self.sosa.Actuator))

        for obj in data.models:
            # Crreate a class for the object type
            obj_class = rdflib.URIRef('http://example.com/object#' + obj.type)
            subj = rdflib.URIRef('http://example.com/object#' + obj.type + str(self.counter))
            
            pos = rdflib.Literal(obj.pose.position)
            size = rdflib.Literal(obj.size)
            orient = rdflib.Literal(obj.pose.orientation)

            self.graph.add((subj, self.rdf.type, obj_class))
            self.graph.add((subj, self.time.hasTime, timestamp))
            self.graph.add((subj, self.dul.hasLocation, pos))
            self.graph.add((subj, self.soma.hasOrientationVector, orient))
            self.graph.add((subj, self.hasSize, size))
            self.counter += 1

        self.timer += 1
        self.recordedGraphs.append((self.graph, ts))
        if self.image is not None and not self.done:
            cv2.imwrite(img_name, self.image)
            rospy.loginfo('Image saved')
        if self.timer == 100:
            self.save_stream()
            rospy.loginfo("Done") 
            self.done = True


    def save_stream(self):
        for kg, ts in self.recordedGraphs:
            filename = str(ts) + '.ttl'
            kg.serialize(destination='streams' + filename, format='turtle')

    def joint_state_callback(self, data):
        if 'pan' in data.name:
            self.panValue = data.position[data.name.index('pan')]
        

    def camera_callback(self, data):
        # Convert image to OpenCV format
        try:
            # Convert the ROS image message to OpenCV format
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Error converting ROS image message to OpenCV format: {}".format(e))
            return
        
if __name__ == '__main__':
    kgbuilder = KnowledgeGraphBuilder()
    kgbuilder.build_stream()