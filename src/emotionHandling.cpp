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

const std::string pathToSounds = ros::package::getPath("mie443_contest3") + "/sounds/"; 

void emotionReaction(sound_play::SoundClient &soundPlayer) {
    static int emotionStep = 0;
    bool goToNextStep = false;

    // Handle appropriate emotion
    if (emotionValue == 0) {
        // Anger

        if (emotionStep == 0) {
            // Roar
            soundPlayer.playWave(pathToSounds + "sound.wav");
            emotionStep++; // Go to next step
        }
        else if (emotionStep == 1) {
            // Ram them in the shin
            goToNextStep = travel(0.5,FAST_MOVE,0,0);
            if (goToNextStep) emotionStep++; // Go to next step once done moving
        }
        else if (emotionStep == 2) {
            // Retreat
            goToNextStep = travel(-0.5,-FAST_MOVE,0,0);

            if (goToNextStep) clearEmotionState(); // Clear emotion reaction once done reaction (movement)
        }

    }
    else if (emotionValue == 1) {
        
    }
    else if (emotionValue == 2) {
        
    }
    else if (emotionValue == 3) {
        
    }
    else if (emotionValue == 4) {
        
    }
    else if (emotionValue == 5) {
        
    }
    else if (emotionValue == 6) {
        
    }


    // Check if we're done processing emotions
    if (emotionValue < 0) {
        emotionStep = 0;
    }
}











// Act whenever we've detected an emotion present
// Currently just states and records the emotion
void emotionCallback(const std_msgs::Int32::ConstPtr& msg) {
    emotionValue = msg->data;
    ROS_INFO("Emotion %d detected (%s). Terminating lifeform.", emotionValue, emotionName[emotionValue]);

    emotionDetected = true;
}

// Returns most recent emotion scanned
int32_t readEmotion(void) {
    //emotionDetected = false; // Record the scan
    return emotionValue;
}

// Returns scan status
bool newEmotion(void) {
    return emotionDetected;
}

// Clear the emotion handler state
void clearEmotionState(void) {
    emotionDetected = false;
    emotionValue = -1;
}

