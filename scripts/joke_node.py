#!/usr/bin/env python

import rospy
from datetime import *
from actionlib import SimpleActionClient
from std_srvs.srv import Empty, EmptyResponse
from strands_executive_msgs import task_utils
from strands_executive_msgs.abstract_task_server import AbstractTaskServer
from mary_tts.msg import maryttsAction


class JokeServer(AbstractTaskServer):
    def __init__(self, interruptible=False, name='tell_joke'):
        super(JokeServer, self).__init__(name, action_type=maryttsAction, interruptible=interruptible)
        self.maximise_duration_delta = rospy.Duration(rospy.get_param('~maximise_duration_delta', 0))

    def create(self, req):
        t = super(JokeServer, self).create(req)
        task_utils.add_time_argument(t, rospy.Time())
        if self.maximise_duration_delta > rospy.Duration(0):
            d = (t.end_before - t.start_after) - self.maximise_duration_delta
            task_utils.add_duration_argument(t, d)
            t.max_duration = d
        else:
            task_utils.add_duration_argument(t, t.max_duration)
        return t

    def execute(self, goal):
        rospy.loginfo("telling a joke")
        self.server.set_succeeded()


if __name__ == '__main__':

    rospy.init_node("joke_node")

    interruptible = rospy.get_param("~interruptible", False)
    # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received

    joker = JokeServer(interruptible=interruptible, name=rospy.get_name())

    rospy.spin()
