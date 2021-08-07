import argparse
# import rospy
from slam.RunMRCLAMDataset import RunMRCLAMDataset
from slam.RunRobotSimulator import RobotSimulatorRunner

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run Different Slam Experiments.")
    parser.add_argument('--name', metavar='E', 
                        help='Experiment name. Ex: MrClam, Sim. This is case insensitive')
    args = parser.parse_args()
    name = args.name.lower()

    if name == None:
        print('Please add a name')
    elif name == 'mrclam':
        clam_dataset = RunMRCLAMDataset(init_landmarks = False, num_particles = 2, num_steps = 10000, hardcode_compass = True)
        clam_dataset.load_data('../../datasets/Jar/dataset1.pkl')
        clam_dataset.run_fast_slam2()
        clam_dataset.plot()
    elif name == 'sim':
        runner = RobotSimulatorRunner()
        runner.run()
    else:
        print('Unknown name')