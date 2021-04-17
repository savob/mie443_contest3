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
extern const char *emotionName[7]; // Emotion descriptions
extern int32_t emotionValue; // Stores most recent scanned emotion (-1 if no emotion found/present)
extern bool emotionDetected; // Has there been a recent (unused) emotion pickup?
// Set to true after a successful scan, set back to false once read is complete

#endif
