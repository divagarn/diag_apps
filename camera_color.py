#!/usr/bin/env python

import rospy
import datetime
from rospy import Time
from sensor_msgs.msg import Image
import argparse

class CameraColorMonitor:
    def __init__(self, total_count_duration):
        rospy.init_node('scan_monitor')

        self.start_time = None
        self.end_time = None
        self.min_hz = 13.0
        self.max_hz = 16.0
        self.success = True
        self.camera_color_count = 0
        self.start_count = 5
        # self.total_count_duration = 20
        self.total_count_duration = total_count_duration
        self.batt_per = None

        if rospy.has_param("/camera_color_params"):
            rospy.delete_param("/camera_color_params")

        self.timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_file_name = f"camera_color_log_{self.timestamp}.txt"

        rospy.Subscriber('/camera/color/image_raw', Image, self.camera_callback)

    def write_log(self, message):
        log_file = open(self.log_file_name, "a")
        log_file.write(f"{message}\n")
        log_file.close()

    def camera_callback(self, msg):
        call_topic = "color_img"
        self.batt_per = rospy.get_param("/haystack/battery_percentage")
        if self.start_time is None:
            self.start_time = Time.now()
            self.end_time = self.start_time + rospy.Duration(self.total_count_duration)

        current_time = Time.now()
        if current_time > self.end_time:
            if self.success:
                rospy.loginfo("Success: Frequency in the range of {} Hz to {} Hz".format(self.min_hz, self.max_hz))
                camera_color_topic_result = "{},{},{},{},{}".format("Success" if self.success else "Failure", self.min_hz, self.max_hz, self.batt_per, call_topic)
                rospy.set_param("/camera_color_params", camera_color_topic_result)
                print(camera_color_topic_result)
            else:
                rospy.loginfo("Failed: Frequency outside the range of {} Hz to {} Hz".format(self.min_hz, self.max_hz))
                camera_color_topic_result = "{},{},{},{},{}".format("Success" if self.success else "Failure", self.min_hz, self.max_hz, self.batt_per, call_topic)
                rospy.set_param("/camera_color_params", camera_color_topic_result)
                print(camera_color_topic_result)

            rospy.signal_shutdown("Monitoring completed")
            rospy.delete_param("/camera_color_params")

        else:
            # Skip the first 5 messages
            if self.camera_color_count >= self.start_count:
                duration = (current_time - self.start_time).to_sec()
                scan_frequency = self.camera_color_count / duration
                self.write_log(scan_frequency)
                #print(scan_frequency, self.camera_color_count)

                if scan_frequency < self.min_hz or scan_frequency > self.max_hz:
                    self.success = False
            self.camera_color_count += 1


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description="Scan Monitor Node")
        parser.add_argument('--total_count_duration', type=float, default=20.0,
                            help="Total count duration in seconds")
        args = parser.parse_args()

        monitor = CameraColorMonitor(args.total_count_duration)
        # monitor = ScanMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
