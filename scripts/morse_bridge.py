#!/usr/bin/python3
import sys
import rospy
from humans_msgs.msg import TwistArray
from humans_msgs.msg import HumanArray
from humans_msgs.msg import HumanMarker
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
        

    def HumanMorsePub(self):

        rospy.init_node('HumanMorseBridge', anonymous=True)
        rate = rospy.Rate(10)
        hum_pos_sub = []
        hum_vel_sub = []

        for human_id in range(1,self.num_hum+1):
            name = "human"+str(human_id)
            self.hum_pub.append(rospy.Publisher("/humans/" + name + "/cmd_vel",Twist,queue_size=1))
            name = 'human'+str(human_id)
            hum_pos_sub.append(message_filters.Subscriber('/humans/' + name + '/pose',PoseStamped))
            hum_vel_sub.append(message_filters.Subscriber('/humans/' + name + '/velocity',TwistStamped))
            humanmarker = HumanMarker()
            humanmarker.id = human_id
            self.humans.humans.append(humanmarker)
        self.humans_pub = rospy.Publisher("/humans/humans", HumanArray, queue_size=1)
        rospy.Subscriber("/humans/cmd_vel", TwistArray, self.HumansCmdVelCB)   
        pose_msg = message_filters.TimeSynchronizer(hum_pos_sub + hum_vel_sub, 10)
        pose_msg.registerCallback(self.HumansPoseCB)
        # vel_msg = message_filters.TimeSynchronizer(hum_vel_sub, 10)
        # pose_msg.registerCallback(self.HumansVelCB)
        srv = Server(HumanConfig, self.DynamicReconfCB)
        rospy.spin()

    def HumansCmdVelCB(self,msg):
        self.hum_cmd_vel = msg
        active_human = 0
        for human in self.humans.humans:
            if(human.active):
                active_human = human.id -1
        self.hum_pub[active_human].publish(self.hum_cmd_vel.twist[active_human])

    def HumansPoseCB(self,*msg):
        for human in self.humans.humans:
            human.pose = msg[human.id-1].pose
            if(human.active):
                human.velocity = msg[human.id+len(self.humans.humans)-1].twist
        self.humans_pub.publish(self.humans)

    # def HumansVelCB(self,*msg):
    #     for human in self.humans.humans:
    #         human.velocity = msg[human.id-1].velocity
    #     self.humans_pub.publish(self.humans)

    def DynamicReconfCB(self, config, level):
        rospy.loginfo("Active Human id is: %d", config.active_human_id)
        for human in self.humans.humans:
            if(human.id == config.active_human_id):
                human.active=True
            else:
                human.active=False
        return config


if __name__ == '__main__':
    human_morse = HumanMorse(num_hum=2)
    human_morse.HumanMorsePub()
    