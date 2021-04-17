#include <ros/ros.h>
#include <ros/package.h>
#include "explore.h"
#include "emotionHandling.h"
#include "movement.h"
#include <time.h>

// If you cannot find the sound play library try the following command.
// sudo apt install ros-kinetic-sound-play
#include <sound_play/sound_play.h>
#include <ros/console.h>

#define EMOTION_BYPASS // Used to skip emotion reactions in testing (comment out before proper runs)

int main(int argc, char** argv) {
    // Monitor time elapsed
    time_t startTime = time(NULL);
    float secondsElapsed = 0;

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
        static bool overridingPrev = false; // Store overriding motion status on previous step

        // Check for emotions
        if (readEmotion() >= 0) {
#ifdef EMOTION_BYPASS
            ROS_WARN("BYPASSING EMOTION CODE");
            clearEmotionState();
#else
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
#endif // EMOTION_BYPASS
        }
        else {
            // We're currently exploring

            if (spinInPlace) {
                // Manually spin in a circle
                manualOverride = true;
                spinInPlace = ((travel(0, 0, M_PI * 2, FAST_SPIN)) == false); 

                // Check if we're done
                if (spinInPlace == false) {
                    ROS_INFO("Done spinning in the spot.");
                    manualOverride = false; // Release control
                }
            }
            else {
                // Check if we've moved and if not spin in place
                if (checkIfMoved() == false) {
                    ROS_WARN("Robot detected as stationary. Initiating spin move.");
                    spinInPlace = true;
                }
                else {
                    // Carry on exploring hero
                }
            }
        }

        // Are we manually controlling robot motion
        if (manualOverride) {
            // Setup control if we just siezed manual control
            if (overridingPrev == false) {
                ROS_WARN("Taking manual control of motion.");
                vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
            }

            // Set motions needed
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
        }
        // Check if we have just released control this loop iteration
        if ((overridingPrev) && (manualOverride == false)) {
            travel(0, 0, 0, 0); // Reset motion code
            vel_pub.shutdown(); // Stop override when no longer needed
            ROS_WARN("Releasing manual control of motion, exploring again.");
            explore.start(); // Explore again once control is released
        }

        overridingPrev = manualOverride; // Record override state for reference next loop
        secondsElapsed = time(NULL) - startTime;
        ros::Duration(0.01).sleep();

        // Check if we're done reacting to all expected victims
        if (victimsEncountered == victimsExpected) {
            ROS_WARN("Interacted with all %d expected victims. Ending search.", victimsExpected);
            break;
        }
    }

    // Output closing messages
    std::cout << std::endl; // Seperate from previous messages
    secondsElapsed = time(NULL) - startTime;
    ROS_INFO("Program execution took %.2f seconds to complete.", secondsElapsed);
    ROS_FATAL("TERMINATING PROGRAM");
    return 0;
}
