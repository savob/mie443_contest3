# MIE443 Contest 3 Code

Group 22:
Savo Bajic - 1003051485
Maximilian Glidden - 1002277396
Catherine Kucaba - 1003278026

## Commands needed to properly execute the robot code
Command lines required to initiate the code are given in sequential order below. They are no different to the standard set in the contest outlines. 
NOTE: Each one of these lines need to be executed in seperate terminal windows, in the order presented below. 
NOTE: the file path at the end of the third command "roslaunch turtlebot_..." needs to be adjusted for the testing computer so it points to the right map file for the contest.

clear; cd ~/catkin_ws/; catkin_make
conda deactivate; roslaunch mie443_contest3 turtlebot_world.launch world:=practice
conda deactivate; roslaunch mie443_contest3 gmapping.launch
conda activate mie443; cd ~/catkin_ws/src/mie443_contest3/src/; python victimLocator.py
cd ~/catkin_ws/src/mie443_contest3/src/; python emotionClassifier.py

roslaunch mie443_contest3 contest3.launch
rosrun sound_play soundplay_node.py

If a visualization of the state of the robot's localization is desired during the run, execute the following command in a seperate terminal either right before "roslaunch mie443_contest3 contest3.launch" or after it.

roslaunch turtlebot_rviz_launchers view_navigation.launch

## Commands required after the code executes
There are no commands needed once execution is complete. However the results can be quickly reached and read in terminal by issuing the following command:

nano ~/catkin_ws/src/mie443_contest3/src/detectedVictim.txt 

## Computer specific file locations in our code
There are no computer specific file locations that need to be modified in our code if our diectory has been downloaded in its entirety to the user's catkin workspace properly. 

## Output file
The output file will be stored in the "src" folder of this package as "detectedVictim.txt", for an overall location of:

~/catkin_ws/src/mie443_contest3/src/detectedVictim.txt
