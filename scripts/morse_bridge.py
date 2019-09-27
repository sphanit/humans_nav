#!/usr/bin/python3
import sys
import rospy
from humans_msgs.msg import TwistArray
from humans_msgs.msg import HumanArray
from humans_msgs.msg import HumanMarkerStamped
from humans_msgs.msg import HumanMarker
from hanp_msgs.msg import TrackedHumans
from hanp_msgs.msg import TrackedHuman
from hanp_msgs.msg import TrackedSegmentType
from hanp_msgs.msg import TrackedSegment
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from dynamic_reconfigure.server import Server
from humans_nav.cfg import HumanConfig
import message_filters


class HumanMorse(object):

    def __init__(self, num_hum):
        self.num_hum = num_hum
        self.humans = HumanArray()
        self.hum_cmd_vel = []
        self.hum_pub = []
        self.humans_pub = []
        self.tracked_humans_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO


    def HumanMorsePub(self):

        rospy.init_node('HumanMorseBridge', anonymous=True)
	    # rate = rospy.Rate(40)
        hum_marker_sub = []

        for human_id in range(1,self.num_hum+1):
            name = "human"+str(human_id)
            self.hum_pub.append(rospy.Publisher("/" + name + "/cmd_vel",Twist,queue_size=1))
            name = 'human'+str(human_id)
            hum_marker_sub.append(message_filters.Subscriber('/' + name, HumanMarkerStamped))
            humanmarker = HumanMarker()
            humanmarker.id = human_id
            self.humans.humans.append(humanmarker)

        self.humans_pub = rospy.Publisher("/humans", HumanArray, queue_size=1)
        self.tracked_humans_pub = rospy.Publisher("/tracked_humans", TrackedHumans, queue_size=1)
        rospy.Subscriber("/humans/cmd_vel", TwistArray, self.HumansCmdVelCB)
        pose_msg = message_filters.TimeSynchronizer(hum_marker_sub, 10)
        pose_msg.registerCallback(self.HumansCB)
        srv = Server(HumanConfig, self.DynamicReconfCB)
        rospy.spin()

    def HumansCmdVelCB(self,msg):
        self.hum_cmd_vel = msg
        active_human = 0
        for human in self.humans.humans:
            if(human.active):
                active_human = human.id -1
                # print(active_human)
        self.hum_pub[active_human].publish(self.hum_cmd_vel.twist[active_human])

    def HumansCB(self,*msg):
        tracked_humans = TrackedHumans()

        for human in self.humans.humans:
            human.pose = msg[human.id-1].human.pose
            human.velocity = msg[human.id-1].human.velocity
            if human.active:
                human_segment = TrackedSegment()
                human_segment.type = self.Segment_Type
                human_segment.pose.pose = human.pose
                human_segment.twist.twist = human.velocity
                tracked_human = TrackedHuman()
                tracked_human.track_id = 3
                tracked_human.segments.append(human_segment)
                tracked_humans.humans.append(tracked_human)

        if(tracked_humans.humans):
            tracked_humans.header.stamp = rospy.Time.now()
            tracked_humans.header.frame_id = 'map'
            self.tracked_humans_pub.publish(tracked_humans)

        self.humans_pub.publish(self.humans)

    def DynamicReconfCB(self, config, level):
        rospy.loginfo("Active Human id is: %d", config.active_human_id)
        for human in self.humans.humans:
            if(human.id == config.active_human_id):
                human.active=True
            else:
                human.active=False
        return config


if __name__ == '__main__':
    human_morse = HumanMorse(num_hum=1)
    human_morse.HumanMorsePub()
