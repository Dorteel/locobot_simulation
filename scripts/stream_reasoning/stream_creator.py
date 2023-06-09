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
        self.onto = rdflib.Graph().parse('thrash.owl')
        self.dir = os.getcwd()
        # Set up cv_bridge for image conversion
        self.bridge = CvBridge()
        # Set up RDF graph and namespaces
        self.owl = rdflib.Namespace("http://www.w3.org/2002/07/owl#")
        self.rdf = rdflib.Namespace("http://www.w3.org/1999/02/22-rdf-syntax-ns#")
        self.rdfs = rdflib.Namespace("http://www.w3.org/2000/01/rdf-schema#")
        self.owl2rl = rdflib.Namespace("http://www.w3.org/2002/03owlt/rl#")
        self.sosa = rdflib.Namespace("http://www.w3.org/ns/sosa/#") 
        self.dul = rdflib.Namespace("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#")
        self.soma = rdflib.Namespace("http://www.ease-crc.org/ont/SOMA.owl#")
        self.time = rdflib.Namespace("https://www.w3.org/TR/2022/CRD-owl-time-20221115#")
        self.locobotNS = rdflib.Namespace('http://example.com/locobot#')

        # Set up some constants
        self.panValue = 0
        self.counter = 0
        self.timer = 0
        self.recordings = 100
        self.done = False
        self.image = None
        self.data = None

        self.kgFormat = 'ttl'
        self.kgPath, self.imgPath = self.create_dir()
        self.recordedGraphs = []

        # Set the publishing rate
        self.rate = rospy.Rate(100) # 1 Hz


    def build_stream(self):
        while not rospy.is_shutdown():
            #rospy.loginfo(self.graph)
            # Sleep to maintain the publishing rate
            self.rate.sleep()


    def construct_graph(self, data):
        graph = rdflib.Graph()
        # Get current time and date
        ts = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")
        # Create filename with current time and date
        timestamp = rdflib.Literal(ts)
        pan_joint_value = rdflib.Literal(self.panValue)
        img_name = os.path.join(self.imgPath, 'image' + str(timestamp) + '.jpg')
        imgNode = rdflib.URIRef('http://example.com/locobot/images#' + img_name)
        #graph.add((timestamp, self.rdf.type, self.sosa.ResultTime))
        
        graph.add((imgNode, self.time.hasTime, timestamp))
        graph.add((self.locobotNS.pan, self.locobotNS.hasJointValue, pan_joint_value))
        graph.add((self.locobotNS.pan, self.rdf.type, self.sosa.Actuator))

        for obj in data.models:
            # Crreate a class for the object type
            obj_class = rdflib.URIRef('http://example.com/object#' + obj.type)
            subj = rdflib.URIRef('http://example.com/object#' + obj.type + str(self.counter))
            
            pos = rdflib.Literal(obj.pose.position)
            size = rdflib.Literal(obj.size)
            orient = rdflib.Literal(obj.pose.orientation)

            graph.add((subj, self.rdf.type, obj_class))
            graph.add((subj, self.locobotNS.seenAt, timestamp))
            graph.add((subj, self.dul.hasLocation, pos))
            graph.add((subj, self.soma.hasOrientationVector, orient))
            graph.add((subj, self.locobotNS.hasSize, size))
            self.counter += 1

        #graphName = 'kg_{}.ttl'.format(str(timestamp))
        graphName = os.path.join(self.kgPath, 'kg_{}.{}'.format(str(timestamp), self.kgFormat))
        
        graph.serialize(graphName, format=self.kgFormat)
        
        if self.image is not None and not self.done:
            cv2.imwrite(img_name, self.image)
            rospy.loginfo('Image saved')

    def create_dir(self):
        # Get the current directory where the script is located
        script_directory = os.path.dirname(os.path.abspath(__file__))
        recordings_directory = os.path.join(script_directory, 'recordings')
        # Generate timestamp
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

        # Create the directory path with timestamp
        directory_path = os.path.join(recordings_directory, timestamp)
        image_path = os.path.join(directory_path, 'images')
        # Check if the directory already exists
        if not os.path.exists(directory_path): os.makedirs(directory_path)
        if not os.path.exists(image_path): os.makedirs(image_path)  
        return directory_path, image_path


    def save_stream(self):
        csv_file = "streams/stream_reasoning_bsc.csv"
        with open(csv_file, mode="w", newline="") as file:
            writer = csv.writer(file)
                
            for kg, ts in self.recordedGraphs:
                for s, p, o in kg:
                    writer.writerow([ts, s.n3(), p.n3(), o.n3()])


    def logical_camera_callback(self, data):
        # Extract information from LogicalCameraImage
        self.data = data
        self.construct_graph(data)

        self.timer += 1
        #self.recordedGraphs.append((self.graph, ts))

        if self.timer == self.recordings:
            #self.save_stream()
            rospy.loginfo("Done") 
            self.done = True


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