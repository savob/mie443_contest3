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
    clearEmotionState();

    // Manual motion setup (used for reactions and recovering exploration)
    ros::Subscriber odom = n.subscribe("odom", 1, &odomCallback);
    ros::Publisher vel_pub;
    geometry_msgs::Twist vel;
    bool spinInPlace = true; // Is rover supposed to be spinning in place
    bool manualOverride = true; // Are we manually overriding the bot's motion
    // Initialised to true since we want the robot to always start with a scan

    // Frontier exploration algorithm.
    explore::Explore explore;

    ROS_WARN("STARTING MAIN LOOP!\n");
    while(ros::ok()) {
        ros::spinOnce();
        static bool overridingPrev = false; // Store if we were overriding motion or not on previous step

        // Check for emotions
        if (readEmotion() >= 0) {
            // Sieze manual control of motion for motional reactions
            if (manualOverride == false) {
                explore.stop();
                manualOverride = true;
            }

            // Handle new emotion
            emotionReaction(sc);

            // Are we done with the remotion state?
            if (readEmotion() < 0) {
                // Release control (return to exploring)
                manualOverride = false;
                ROS_INFO("Done emotional reaction");
            }
        }
        else {
            // We're currently exploring
            if (spinInPlace) {
                // Manually spin in a circle
                manualOverride = true;
                spinInPlace = ((travel(0, 0, M_PI * 2, FAST_SPIN)) == false); 

                // Check if we're done
                if (spinInPlace == false) {
                    ROS_INFO("Done spinning in the spot. Emotion %d", readEmotion());
                    manualOverride = false; // Release control
                }
            }

            // TODO: add code to monitor distance travelled over last period and see if it is enough motion if not, do a spin
        }

        // Are we manually controlling robot motion
        if (manualOverride) {
            // Setup control if we just siezed manual control
            if (overridingPrev == false) {
                ROS_INFO("Taking manual control of motion.");
                vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
            }

            // Set motions needed
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
        }
        // Check if we have just released control this loop iteration
        if ((overridingPrev) && (manualOverride == false)) {
            vel_pub.shutdown(); // Stop override when no longer needed
            ROS_INFO("Releasing manual control of motion, exploring again.");
            explore.start(); // Explore again once control is released
        }

        overridingPrev = manualOverride; // Record override state for reference
        ros::Duration(0.01).sleep();
    }
    return 0;
}
