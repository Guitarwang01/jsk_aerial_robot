#!/usr/bin/env python3
"""Isolated ROS transport smoke test; synthetic input/model, no hardware nodes."""
import os
from pathlib import Path
import subprocess
import sys
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11427'
os.environ['ROS_IP'] = '127.0.0.1'
os.environ['ROS_HOME'] = '/tmp/fish-acoustic-ros-smoke'
os.environ['ROS_LOG_DIR'] = '/tmp/fish-acoustic-ros-smoke/log'
sys.path.insert(0, str(ROOT / 'scripts'))


def main():
    master = subprocess.Popen(['roscore', '-p', '11427'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(100):
            if master.poll() is not None:
                raise RuntimeError('isolated roscore failed to start')
            try:
                xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/smoke')
                break
            except OSError:
                time.sleep(.1)
        else:
            raise RuntimeError('roscore startup timeout')
        import rospy
        from robotic_fish_io.msg import AdcSample, AdcSampleArray
        from robotic_fish_acoustic.msg import QuadrantProbabilities
        from quadrant_node import Node
        rospy.init_node('quadrant_smoke', disable_signals=True)
        rospy.set_param('~model', str(ROOT / 'test/fixture_model.json'))
        node = Node()
        received = []
        sub = rospy.Subscriber('~probabilities', QuadrantProbabilities, received.append)
        pub = rospy.Publisher('/robotic_fish/adc/samples', AdcSampleArray, queue_size=10)
        deadline = time.monotonic() + 5
        while (pub.get_num_connections() == 0 or node.pub.get_num_connections() == 0) and time.monotonic() < deadline:
            time.sleep(.02)
        for _ in range(12):
            message = AdcSampleArray()
            for channel, voltage in enumerate([1., 2., 1.]):
                sample = AdcSample()
                sample.timestamp = rospy.Time.now()
                sample.channel_id = channel
                sample.volt_cali = voltage
                sample.volt_raw = voltage
                sample.status_cali = False
                sample.status_dac_feedback = True
                sample.dac_volt = .9
                message.samples.append(sample)
            pub.publish(message)
            time.sleep(.05)
        time.sleep(.4)
        updates = [r for r in received if r.updated]
        assert len(updates) == 12, len(updates)
        last = updates[-1]
        assert last.header.frame_id == 'fish_base_link'
        assert last.status & last.CALIBRATION_FALLBACK
        assert abs(last.probability_left + last.probability_right - 1) < 1e-12
        assert abs(last.probability_left - last.probability_front_left - last.probability_rear_left) < 1e-12
        held = received[-1]
        assert not held.updated and held.status & held.STALE
        assert held.estimate_stamp == last.estimate_stamp
        print('PASS: 12 ROS updates, fallback retained, FRD marginals, stale heartbeat retains estimate stamp')
        sub.unregister()
        pub.unregister()
        node.timer.shutdown()
        rospy.signal_shutdown('test complete')
    finally:
        master.terminate()
        try:
            master.wait(timeout=10)
        except subprocess.TimeoutExpired:
            master.kill()
            master.wait()


if __name__ == '__main__':
    main()
