#!/usr/bin/env python

__author__ = "Martin Pecka <martin.pecka@cvut.cz>"

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest


class LatchHelper(object):
    """Helper class that calls the publish_latched_messages service of a remote sender whenever a receiver gets ready."""

    def __init__(self):

        # name of the remote service to be called
        self._service_server_name = rospy.get_param("~service_server_name")

        self._ready_subscriber = rospy.Subscriber("/network/receiver_ready",
                                                  String, self.receiver_ready_cb, queue_size=1)

        self._service_server = rospy.ServiceProxy(self._service_server_name + "/publish_latched_messages", Empty)

    def receiver_ready_cb(self, receiver_name_msg):
        try:
            self._service_server.wait_for_service(timeout=0.1)
            self._service_server(EmptyRequest())
        except rospy.ROSException:  # It means the waiting timed out
            # So we have nobody to contact and we can omit calling the service
            pass


if __name__ == '__main__':
    rospy.init_node("tcp_receiver_latch_helper")
    helper = LatchHelper()
    rospy.spin()
