#ifndef EMOTION_HANDLING_GROUP22_H
#define EMOTION_HANDLING_GROUP22_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h> // Emotion messages

void emotionCallback (const std_msgs::Int32::ConstPtr& msg);
int32_t readEmotion(void); // Returns most recent emotion scanned and records the read
bool newEmotion(void); // Returns if there has been a recent (unread) scan

/*
    0=Angry
    1=Disgust
    2=Fear
    3=Happy
    4=Sad
    5=Surprise
    6=Neutral
*/

/* 
    Can't include variable here as globals for some reason keeps causing issues 
    due to "multiple declarations" for some reason if any variables are declared
    in this header. Hopefully not any other headers I introduce will have the same 
    issue when compiling.
*/

#endif
