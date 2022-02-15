#!/usr/bin/env python
# Show how to publish Float32MultiArray

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension


class MultiArrayPublisher(object):
    def __init__(self):
        # number of values
        self.pub = rospy.Publisher('float_array', Float32MultiArray, queue_size=2)
        self.update = rospy.Timer(rospy.Duration(0.1), self.update)

    def update(self, event):
        # make an interesting changing signal
        cur_sec = event.current_expected.to_sec()
        amplitude = np.cos(cur_sec * 2.4)
        freq = 5.0 + 10.0 * np.sin(cur_sec * 1.0)
        sig_t = np.arange(0.0, 1.0, 0.01, dtype=np.float32)
        sig = amplitude * np.sin(sig_t * freq * 2.0 * np.pi)
        # put a sin envelop around the whole thing
        sig *= np.sin(sig_t / np.max(sig_t) * 2.0 * np.pi)

        msg = Float32MultiArray()
        msg.data = sig.flatten().tolist()

        # TODO(lucasw) also put time values into data if graph_rviz_plugin can use it

        # TODO(lucasw) is this needed at all?
        # does graph_rviz_plugin use it?  No it doesn't
        if False:
            dim = MultiArrayDimension()
            dim.label = "sine wave"
            dim.size = len(msg.data)
            dim.stride = dim.size
            msg.layout.dim.append(dim)

        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('multi_array_publisher')
    node = MultiArrayPublisher()
    rospy.spin()
