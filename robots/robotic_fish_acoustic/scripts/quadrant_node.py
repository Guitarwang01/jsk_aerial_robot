#!/usr/bin/env python3
"""Publish results even when quality flags are present; never refresh held stamps."""
import threading
import rospy
from robotic_fish_io.msg import AdcSampleArray
from robotic_fish_acoustic.msg import QuadrantProbabilities
from robotic_fish_acoustic.core import CLASSES, Estimator, Model


class Node:
    def __init__(self):
        path = rospy.get_param('~model', '')
        self.engine = Estimator(Model(path) if path else None,
                                rospy.get_param('~window_s', .5),
                                rospy.get_param('~gap_s', .2),
                                rospy.get_param('~low_signal_v', .001))
        self.lock = threading.Lock()
        self.pub = rospy.Publisher('~probabilities', QuadrantProbabilities, queue_size=20)
        self.sub = rospy.Subscriber('/robotic_fish/adc/samples', AdcSampleArray, self.receive, queue_size=100)
        self.timer = rospy.Timer(rospy.Duration(.1), self.tick, reset=True)
        if not path:
            rospy.logwarn('No trained quadrant model: features are processed, probabilities remain unavailable.')

    def publish(self, result):
        msg = QuadrantProbabilities()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'fish_base_link'
        for key in ('estimate_stamp', 'window_start', 'window_end'):
            setattr(msg, key, rospy.Time.from_sec(result[key] or 0.))
        probabilities = result['probabilities'] or [float('nan')] * 4
        for name, value in zip(CLASSES, probabilities):
            setattr(msg, 'probability_' + name, value)
        msg.probability_left = probabilities[2] + probabilities[3]
        msg.probability_right = probabilities[0] + probabilities[1]
        for key in ('has_estimate', 'updated', 'status', 'estimate_status', 'valid_sample_count', 'model_id', 'timestamp_semantics'):
            setattr(msg, key, result[key])
        self.pub.publish(msg)

    def receive(self, msg):
        with self.lock:
            self.publish(self.engine.push(msg, rospy.Time.now().to_sec()))

    def tick(self, event):
        with self.lock:
            self.publish(self.engine.snapshot(rospy.Time.now().to_sec()))


if __name__ == '__main__':
    rospy.init_node('acoustic_quadrants')
    Node()
    rospy.spin()
