#ifndef EMOTION_HANDLING_GROUP22_H
#define EMOTION_HANDLING_GROUP22_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h> // Emotion messages
#include "movement.h"
#include <sound_play/sound_play.h>

#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui.hpp"

void emotionCallback (const std_msgs::Int32::ConstPtr& msg);
int32_t readEmotion(void); // Returns most recent emotion scanned
void clearEmotionState(void); // Clears emotion handler
void emotionReaction(sound_play::SoundClient &sc); // Handles emotion reactions

void showImage(std::string fileLocation); // Show an image on screen

/*
    0=Angry
    1=Disgust
    2=Fear
    3=Happy
    4=Sad
    5=Surprise
    6=Neutral
*/
extern const char *emotionName[7]; // Emotion descriptions
extern int32_t emotionValue; // Most recent scanned emotion (-1 if no emotion found/present)
extern int victimsEncountered; // Number of victims encountered thus far
extern const int victimsExpected; // Expected victim count

#endif
