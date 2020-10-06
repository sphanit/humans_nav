#!/usr/bin/python
import rospy
import numpy as np
from optitrack_ros.msg import or_pose_estimator_state
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped
import copy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from hanp_msgs.msg import TrackedHumans
from hanp_msgs.msg import TrackedHuman
from hanp_msgs.msg import TrackedSegmentType
from hanp_msgs.msg import TrackedSegment
from humans_msgs.msg import TwistArray
from humans_msgs.msg import HumanArray
from humans_msgs.msg import HumanMarkerStamped
from humans_msgs.msg import HumanMarker


class OptiTrackHuman():

    def __init__(self):
        
        self.pos = Pose()
        self.vel = Twist()
        self.first = True
        self.humans = TrackedHumans()
        self.uwds_humans = HumanArray()
        
        for i in range(1):
            human_segment = TrackedSegment()
            human_segment.type = TrackedSegmentType.TORSO
            tracked_human = TrackedHuman()
            tracked_human.track_id = i+1
            tracked_human.segments.append(human_segment)
            self.humans.humans.append(tracked_human)

            humanmarker = HumanMarker()
            humanmarker.id = i+1
            self.uwds_humans.humans.append(humanmarker)


    def OptiTrackCB(self, msg): 
        if(msg.pos):
            self.pos.position.x = copy.copy(msg.pos[0].x)+6.4868
            self.pos.position.y = copy.copy(msg.pos[0].y)+2.8506
            self.pos.position.z = copy.copy(msg.pos[0].z)

            self.pos.orientation.x = 0#copy.copy(msg.att[0].qx)
            self.pos.orientation.y = 0#copy.copy(msg.att[0].qy)
            self.pos.orientation.z = copy.copy(msg.att[0].qz)
            self.pos.orientation.w = copy.copy(msg.att[0].qw)

            if self.first: 
                self.first=False
                self.prev_pos = copy.deepcopy(self.pos)

            now = rospy.Time.now()
            time = (now-self.last_time).to_nsec()/(10.0**9)
            if time>=0.0:
                self.vel.linear.x = (self.pos.position.x - self.prev_pos.position.x)/time
                self.vel.linear.y = (self.pos.position.y - self.prev_pos.position.y)/time
                self.vel.linear.z = (self.pos.position.z - self.prev_pos.position.z)/time

                quat = (msg.att[0].qx,msg.att[0].qy,msg.att[0].qz,msg.att[0].qw)
                r,p,y = euler = euler_from_quaternion(quat)

                quat_prev = (self.prev_pos.orientation.x,self.prev_pos.orientation.y,self.prev_pos.orientation.z,self.prev_pos.orientation.w)
                rp,pp,yp = euler = euler_from_quaternion(quat_prev)

                self.vel.angular.x = (r - rp)/time
                self.vel.angular.y = (p - pp)/time
                self.vel.angular.z = (y - yp)/time
        

            self.last_time = now
            self.prev_pos = copy.deepcopy(self.pos)

        for human in self.humans.humans:
            for segment in human.segments:
                if segment.type == TrackedSegmentType.TORSO and human.track_id==1:
                    segment.pose.pose = self.pos
                    segment.twist.twist = self.vel

        for human in self.uwds_humans.humans:
            human.pose = self.pos
            human.velocity = self.vel

        self.publishHumans()

    def publishHumans(self):
        
        if(self.humans.humans):
            self.humans.header.stamp = rospy.Time.now()
            self.humans.header.frame_id = 'map'
            self.pub.publish(self.humans)
            self.uwds_pub.publish(self.uwds_humans)
            #print('updated')
            #print(self.humans)

    def run_and_publish(self):
        
        rospy.init_node('mocap_humans')
        #rospy.Subscriber('/optitrack/bodies/Rigid_Body_1',or_pose_estimator_state,self.OptiTrackCB)
        rospy.Subscriber('/optitrack/bodies/Helmet_3',or_pose_estimator_state,self.OptiTrackCB)
        self.pub = rospy.Publisher('/tracked_humans', TrackedHumans, queue_size=10)
        self.uwds_pub = rospy.Publisher('/humans', HumanArray, queue_size=1)
        self.last_time = rospy.Time.now()
        rospy.spin()

a = OptiTrackHuman()
a.run_and_publish()
