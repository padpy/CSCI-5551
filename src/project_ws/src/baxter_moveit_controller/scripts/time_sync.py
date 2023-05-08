#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo
import os
import time
import sys

secs = 0
nsecs = 0

class TimeSync:
    def __init__(self, topic, clock_offset=0.1, sync_interval=600) -> None:
        self.topic = topic
        self.clock_offset = clock_offset
        self.sync_interval = sync_interval

        self.secs = 0
        self.nsecs = 0
        self._synced = False
        self._last_sync = 0

        rospy.Subscriber(self.topic, CameraInfo, self._topic_callback, queue_size=1)
        rospy.init_node('client_time_sync', anonymous=True)

        self._cycle_rate = rospy.Rate(1000)

    def run(self):
        while not rospy.is_shutdown():
            now = self.time_from_epoches(self.secs, self.nsecs)
            if (not self._synced and now) or  now - self._last_sync > self.sync_interval:
                self._set_time(self.secs, self.nsecs)
                self._last_sync = now
                self._synced = True
            
            self._cycle_rate.sleep()

    def _set_time(self, secs, nsecs):
        nsecs = (self.clock_offset % 1) * 1e9 + nsecs
        secs = self.secs + nsecs // 1e9 + self.clock_offset // 1
        nsecs = nsecs % 1e9
        os.system(f"sudo date -s '@{int(secs)}.{int(nsecs):09}'")

    def _topic_callback(self, msg: CameraInfo):
        stamp = msg.header.stamp
        self.secs = stamp.secs
        self.nsecs = stamp.nsecs

    def time_from_epoches(self, secs, nsecs):
        return secs + nsecs / 1e9

        
if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise RuntimeError("Must provide rostopic to listen to: TOPIC [CLOCK_OFFSET CLOCK_SYNC_INTERVAL]")
    topic = sys.argv[1]
    clock_offset = 0.1 if len(sys.argv) < 3 else sys.argv[2]
    sync_interval = 600 if len(sys.argv) < 4 else sys.argv[3]
    timeSync = TimeSync(topic, clock_offset, sync_interval)
    timeSync.run()