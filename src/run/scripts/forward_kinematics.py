#!/usr/bin/env python

import numpy
import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf.msg
from urdf_parser_py.urdf import URDF

"""This function will transform a 4x4 transformation matrix T into a ros message 
which can be published. In addition to the transform itself, the message
also specifies who is related by this transform, the parent and the child.
It is an optional function which you may use as you see fit."""
def convert_to_message(T, child, parent):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child
    translation = tf.transformations.translation_from_matrix(T)
    rotation = tf.transformations.quaternion_from_matrix(T)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]        
    return t
    
#Our main class for computing Forward Kinematics
class ForwardKinematics(object):

    #Initialization
    def __init__(self):
        """Announces that it will publish forward kinematics results, in the form of tfMessages.
        "tf" stands for "transform library", it's ROS's way of communicating around information
        about where things are in the world"""
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)



    def callback(self, joint_values):
        link = self.robot.get_root()
        worldlink=link
        trans = tf.msg.tfMessage()
        T = tf.transformations.identity_matrix()
        while True:
            if link in self.robot.child_map:
                (next_joint_name, next_link) = self.robot.child_map[link][0]
            else:
                break
            next_joint = self.robot.joint_map[next_joint_name]
            print next_joint
            t = tf.transformations.translation_matrix(next_joint.origin.xyz)
            if next_joint_name in joint_values.name:
                position = joint_values.position[joint_values.name.index(next_joint.name)]
            else:
                position = 0
            if next_joint.type == 'revolute':
                R1 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis \
                (position, next_joint.axis))
            if next_joint.type == 'fixed':
                R1 = tf.transformations.identity_matrix()
            R2 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler \
            (next_joint.origin.rpy[0],next_joint.origin.rpy[1],next_joint.origin.rpy[2]))
            R = numpy.dot(R1,R2)
            T=numpy.dot(T,numpy.dot(t,R))
            trans.transforms.append(convert_to_message(T, next_link, worldlink))
            link=next_link
        self.pub_tf.publish(trans)

            
        

if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
    rospy.spin()

