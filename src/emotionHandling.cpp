#include "emotionHandling.h"

/*
    0=Angry
    1=Disgust
    2=Fear
    3=Happy
    4=Sad
    5=Surprise
    6=Neutral
*/

int32_t emotionValue; // Stores most recent scanned emotion
bool emotionDetected; // Has there been a recent (unused) emotion pickup?
// Set to true after a successful scan, set back to false once read is complete

// Act whenever we've detected an emotion present
// Currently just states and records the emotion
void emotionCallback(const std_msgs::Int32::ConstPtr& msg) {
    emotionValue = msg->data;
    ROS_INFO("Emotion %d detected. Terminating lifeform.", emotionValue);

    emotionDetected = true;
}

// Returns most recent emotion scanned and records the read
int32_t readEmotion(void) {
    emotionDetected = false;
    return emotionValue;
}

// Returns scan status
bool newEmotion(void) {
    return emotionDetected;
}