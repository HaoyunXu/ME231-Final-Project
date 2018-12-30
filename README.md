# ME231-Final-Project
Receding Horizon Control Problem for Tracking MPC

Presentation: https://www.youtube.com/watch?v=wu1DD2hRdIk&feature=youtu.be&fbclid=IwAR2sD4Db17CvnKTFos6T7hKTfdVEQvkHxgCItDith9pXWfMoN8YT99xr_Y8

Paper:https://drive.google.com/file/d/1vmT7Zn7i7F1dgnTGh_XUvd1yitjeHPfD/view?usp=sharing









HOW to connect Linux Machine to MATLAB ROS Master:

Step 1: Figure out the IP address on the windows machine running MATLAB.  (mat=the IP of the machine running matlab)
Step 2: Check IP of the linux machine with hostname -I command. (refer the IP as lin in future tutorial)
Step 3: On linux machine start a new terminal and do following set ups:

export ROS_IP=lin
export ROS_MASTER_URI=http://mat:11311


Step 4: Initiate ROS on matlab with "rosinit" command

Step 5: Then, on the linux machine, under the same terminal that you exported ROS_IP, run the simulator: rosrun hiperlab_rostool mpcratescontrol (vehiclenumber)

