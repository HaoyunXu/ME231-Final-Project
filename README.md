# ME231-Final-Project
Receding Horizon Control Problem for Tracking MPC

HOW to connect Linux Machine to MATLAB ROS Master:

Step 1: Figure out the IP address on the windows machine running MATLAB.  (refer it to mat in future tutorial)
Step 2: Check IP of the linux machine with hostname -I command. (refer the IP as lin in future tutorial)
Step 3: On linux machine start a new terminal and do following set ups:

export ROS_IP=lin
export ROS_MASTER_URI=http://mat:11311


Step 4: Initiate ROS on matlab with "rosinit" command

Step 5: Then, on the linux machine, under the same terminal that you exported ROS_IP, run the simulator: rosrun hiperlab_rostool mpcratescontrol (vehiclenumber)

