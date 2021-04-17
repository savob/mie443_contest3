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

const char *emotionName[7] = { "Angry", "Disgust", "Fear", "Happy", "Sad", "Surprise", "Neutral"}; // Emotion descriptions

int32_t emotionValue = -1; // Stores most recent scanned emotion (-1 if no emotion found/present)
bool emotionDetected; // Has there been a recent (unused) emotion pickup?
// Set to true after a successful scan, set back to false once read is complete

// Act whenever we've detected an emotion present
// Currently just states and records the emotion
void emotionCallback(const std_msgs::Int32::ConstPtr& msg) {
    emotionValue = msg->data;
    ROS_INFO("Emotion %d detected (%s). Terminating lifeform.", emotionValue, emotionName[emotionValue]);

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