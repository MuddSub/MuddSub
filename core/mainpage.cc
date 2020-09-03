/*! \mainpage MuddSub Softwware
MuddSub is a student-lead team which competes in the international [robosub](http://robosub.org) competition.
This repository represents a fresh start on the code base, and is thus incomplete. Further, the documentation is equally incomplete.
If you have question about code which has not yet been migrated to this repo, or about any missing documentation, contact
Seth Isaacson at sisaacson@hmc.edu.
Our software is partitioned as follows:
### Core
The Core repository provides simple tools for sharing information, managing the robot's state, and other utilitites. It also holds the URDF descirptions of the robot.
### Vision
Vision holds all of our commputer vision tools.
### controls
Controls provides the vehicle dynamics and control systems.
### hydrophones
The Hydrophones repository provides the firmware and software for processing acoustic signals. The overall goal is to find the direction-of-arrival of an acoustic
signal by receiving it on four hydrophones.
### motion planning
Motion planning generates abstract trajectories which the robot follows.
### sim
Sim holds packages to interface with our simulator, which is based on the open-source UWSim. The controls repository also contains code to simulate vehicle dynamics.
### slam
SLAM will eventually hold code to conduct simulataneous localization and mapping.
### thirdparty
We use various third-party tools to service our autonomy stack. Notably, we use the wonderful controls toolbox, which provides interfaces for system linearization and optimal controllers.
### builds
Builds holds three metapackages which are used for building different configurations.
- muddsub_primary builds code which is run on the robot.
- muddsub_sim builds muddsub_primary, plus simulation-related code.
- muddsub_raspi builds the code which runs raspberry pis.
*/
