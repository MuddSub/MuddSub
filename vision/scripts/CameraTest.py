from vision.Camera import Camera
import rospy

if __name__ == '__main__':
    obstacle_names = rospy.get_param("names")
    obstacle_names = [n.strip() for n in obstacle_names.split('\n')]

    model_path = rospy.get_param("model_path")
    model_config_path = rospy.get_param("model_config_path")
    test = Camera('usb_cam', obstacle_names, model_path, model_config_path)
    test.main()
