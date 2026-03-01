#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import argparse
import yaml
import sys

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


if __name__ == "__main__":
    rospy.init_node('dump_params', anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("namespace")
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])
    rospy.spin()
    params = rospy.get_param(args.namespace)
    rospy.loginfo("Saving params from namespace: {}".format(args.namespace))
    with open(args.output, "w") as file_path:
        yaml.dump(params, file_path)
