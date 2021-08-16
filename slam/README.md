# Description
This branch includes FastSLAM 2.0, which also supports FastSLAM 1.0, and two experiments/simulations.

## What is new?
The most recent pull request added

- A FastSLAM package that supports 
  - (1) both FastSLAM 1.0 and FastSLAM 2.0 algorithms. 
  - (2) different robot physics models.  
  - The main feature will be FastSLAM 2.0; hence the name. 
- Currently, a 2D physics model is included.
- A runner on MR.CLAM dataset
- A simulator with adoptable robot physics model and a basic control, and a runner on the simulator. 

## How to run the code?
- This package is located under Muddsub/slam/src. All paths reference will assume the user is at this location. 
- Run catkin build slam. This is required to build slam as a package. Remember to source develop/setup.bash
- For first time user, run MRCLAMDataloader under slam/mrclam
- Run either of the following scripts:
  - RunMRCLAMDataset.py under slam/mrclam
  - RunRobotSimulator.py under slam/simulation

## Component diagram
![image](https://user-images.githubusercontent.com/43415086/128655282-d182ad67-151c-4360-82fe-2e9fbfdd639b.png)

## Next steps
- Expand to include a 3D physics model.
- Expand to include different controls for the simulator
- Expand to include ROS subscription and publishing
# Testing
Reasonable results from both Mr.CLAM experiment and the simulation experiment.

## Comparison between FastSLAM 1.0 and FastSLAM 2.0 on MR.CLAM dataset
- Low robot pose variance:

![low_variance](https://user-images.githubusercontent.com/43415086/128771016-d36f2ad1-4a00-454c-83bb-0213f816768b.png)

- Medium pose variance:

![medium_var](https://user-images.githubusercontent.com/43415086/128771229-fe9feae8-6e67-46f9-9a01-cb29a3cf80a5.png)

- High pose variance:

![high_var](https://user-images.githubusercontent.com/43415086/128771248-1013ccb7-8551-4564-92e1-c6b642ac629c.png)

Please note that landmark offsets can be caused by shifting

# FAQ
- What is the change to README about?
  - This PR temporarily reverted some README change, since the base branch does not use noetic yet.
- What are the *.dat files?
  - This is the data file for Mr.CLAM dataset.  
