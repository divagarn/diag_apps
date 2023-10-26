#!/usr/bin/env python
import datetime
import rospy
from rospy import Time
from sensor_msgs.msg import LaserScan
import argparse


class ScanMonitor:
    def __init__(self, total_count_duration):
        rospy.init_node('scan_monitor')

        self.start_time = None
        self.end_time = None
        self.min_hz = 12.0
        self.max_hz = 17.0
        self.success = True
        self.scan_count = 0
        self.start_count = 5
        self.total_count_duration = total_count_duration
        self.batt_per = None

        if rospy.has_param("/scan_params"):
            rospy.delete_param("/scan_params")

        self.timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_file_name = f"lidar_log_{self.timestamp}.txt"

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def write_log(self, message):
        log_file = open(self.log_file_name, "a")
        log_file.write(f"{message}\n")
        log_file.close()

    def scan_callback(self, msg):
        call_topic = "scan"
        self.batt_per = rospy.get_param("/haystack/battery_percentage")
        if self.start_time is None:
            self.start_time = Time.now()
            self.end_time = self.start_time + rospy.Duration(self.total_count_duration)

        current_time = Time.now()
        if current_time > self.end_time:
            if self.success:
                rospy.loginfo("Success: Frequency in the range of {} Hz to {} Hz".format(self.min_hz, self.max_hz))
            else:
                rospy.loginfo("Failed: Frequency outside the range of {} Hz to {} Hz")

            scan_params_str = "{},{},{},{},{}".format("Success" if self.success else "Failure", self.min_hz, self.max_hz, self.batt_per, call_topic)
            rospy.set_param("/scan_params", scan_params_str)
            # print(scan_params_str)

            rospy.signal_shutdown("Monitoring completed")
            rospy.delete_param("/scan_params")

        else:
            # Skip the first 5 messages
            if self.scan_count >= self.start_count:
                duration = (current_time - self.start_time).to_sec()
                scan_frequency = self.scan_count / duration
                # print(scan_frequency)
                self.write_log(scan_frequency)

                if scan_frequency < self.min_hz or scan_frequency > self.max_hz:
                    self.success = False
            self.scan_count += 1


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description="Scan Monitor Node")
        parser.add_argument('--total_count_duration', type=float, default=20.0,
                            help="Total count duration in seconds")
        args = parser.parse_args()

        monitor = ScanMonitor(args.total_count_duration)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
