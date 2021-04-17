#include <ros/ros.h>
#include <ros/package.h>
#include "explore.h"
#include "emotionHandling.h"
#include "movement.h"

// If you cannot find the sound play library try the following command.
// sudo apt install ros-kinetic-sound-play
#include <sound_play/sound_play.h>
#include <ros/console.h>

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest3");
    ros::NodeHandle n;
    
    // Class to handle sounds.
    sound_play::SoundClient sc;
    
    // The code below shows how to play a sound.
    std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    sc.playWave(path_to_sounds + "sound.wav");
    
    // Emotion node
    ros::Subscriber emotionSub = n.subscribe("/detected_emotion", 1, &emotionCallback);
    int emotionStep = 0;

    // Manual motion setup (used for reactions and recovering exploration)
    ros::Subscriber odom = n.subscribe("odom", 1, &odomCallback);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel;
    bool spinInPlace = true; // Is rover supposed to be spinning in place
    // Initialised to true since we want the robot to always start with a scan

    // Frontier exploration algorithm.
    explore::Explore explore;

    ROS_WARN("STARTING MAIN LOOP!\n");
    while(ros::ok()) {
        ros::spinOnce();

        if (spinInPlace) {
            // Manually spin in a circle
            spinInPlace = ~travel(0, 0, M_PI * 2, SLOW_SPIN); 

            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);

            // Check if we're done
            if (~spinInPlace) {
                if (readEmotion() < 0) {
                    explore.start();    // Explore if there isn't an emotional reposne to continue along
                }
            }
        }


        // Check for emotions
        if (readEmotion() >= 0) {
            explore.stop();

            int emotionIn = readEmotion();
            // Handle new emotion


            // Are we done with the remotion state?
            if (readEmotion() < 0) {
                // If we are done with the emotion state, return to exploring
                emotionStep = 0;
                explore.start();
            }
            else {
                // We're moving manually
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
            }
        }
        else {
            // We're currently exploring

            // TODO: add code to monitor distance travelled over last period and see if it is enough motion if not, do a spin
        }


        ros::Duration(0.01).sleep();
    }
    return 0;
}
