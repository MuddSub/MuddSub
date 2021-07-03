import rospy
import argparse
import yaml
import sys

if __name__ == "__main__":
    rospy.init_node('dump_params', anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("namespace")
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])
    rospy.loginfo("dump_params -___")
    rospy.loginfo("args " + str(args))
    rospy.spin()
    params = rospy.get_param(args.namespace)
    with open(args.output, "w") as file_path:
        yaml.dump(params, file_path)

