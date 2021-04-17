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

// Globals
int victimsEncountered = 0; // Number of victims encountered thus far
const int victimsExpected = 7; // Expected victim count for environment

const char *emotionName[7] = { "Angry", "Disgust", "Fear", "Happy", "Sad", "Surprise", "Neutral"}; // Emotion descriptions

int32_t emotionValue = -1; // Stores most recent scanned emotion (-1 if no emotion found/present)

const std::string pathToSounds = ros::package::getPath("mie443_contest3") + "/sounds/"; 
const std::string pathToImages = ros::package::getPath("mie443_contest3") + "/images/"; 

void emotionReaction(sound_play::SoundClient &soundPlayer) {
    static int emotionStep = 0;
    bool goToNextStep = false;

    const int soundPause = 5; // Length to sleep after each audio track

    // Handle appropriate emotion
    if (emotionValue == 0) {
        // Respond to anger with resentment

        if (emotionStep == 0) {
            soundPlayer.playWave(pathToSounds + "resentment1.wav");
            showImage(pathToImages + "resentment.png");
            sleep(soundPause);
            emotionStep++; // Go to next step
        }
        else if (emotionStep == 1) {
            // Turn around
            goToNextStep = travel(0,0,M_PI,FAST_SPIN);
            if (goToNextStep) emotionStep++; // Go to next step once done moving
        }
        else if (emotionStep == 2) {
            goToNextStep = travel(0.3,FAST_MOVE,0,0);
            if (goToNextStep) emotionStep++;
        }
        else if (emotionStep == 3) {
            // Turn around
            goToNextStep = travel(0,0,DEG2RAD(-100),-SLOW_SPIN);
            if (goToNextStep) emotionStep++;
        }
        else if (emotionStep == 4) {
            soundPlayer.playWave(pathToSounds + "resentment2.wav");
            sleep(soundPause);
            clearEmotionState(); // Clear emotion reaction once done reaction (movement)
        }

    }
    else if (emotionValue == 1) {
        // Respond to disgust with discontent

        if (emotionStep == 0) {
            soundPlayer.playWave(pathToSounds + "discontent1.wav");
            showImage(pathToImages + "discontent.png");
            sleep(soundPause);
            emotionStep++; // Go to next step
        }
        else if (emotionStep == 1) {
            // Go backwards
            goToNextStep = travel(0.25,-SLOW_MOVE,0,0);
            if (goToNextStep) emotionStep++; // Go to next step once done moving
        }
        else if (emotionStep == 2) {
            // Turn around
            goToNextStep = travel(0,0,M_PI,SLOW_SPIN);
            if (goToNextStep) emotionStep++;
        }
        else if (emotionStep == 3) {
            soundPlayer.playWave(pathToSounds + "discontent2.wav");
            sleep(soundPause);
            clearEmotionState(); // Clear emotion reaction once done reaction (movement)
        }
    }
    else if (emotionValue == 2) {
        // Respond to fear with embarassment

        if (emotionStep == 0) {
            soundPlayer.playWave(pathToSounds + "embarrassment1.wav");
            showImage(pathToImages + "embarrassment.png");
            sleep(soundPause);
            emotionStep++; // Go to next step
        }
        else if (emotionStep == 1) {
            // Go backwards
            goToNextStep = travel(-0.4,-SLOW_MOVE,0,0);
            if (goToNextStep) emotionStep++; // Go to next step once done moving
        }
        else if (emotionStep == 2) {
            // Turn around a bit
            goToNextStep = travel(0,0,DEG2RAD(45),-SLOW_SPIN);
            if (goToNextStep) emotionStep++;
        }
        if (emotionStep == 3) {
            soundPlayer.playWave(pathToSounds + "embarrassment2.wav");
            sleep(soundPause);
            emotionStep++; // Go to next step
        }
        else if (emotionStep == 4) {
            // Turn around completely
            goToNextStep = travel(0,0,DEG2RAD(135),-SLOW_SPIN);
            if (goToNextStep) clearEmotionState();
        }
        
    }
    else if (emotionValue == 3) {
        // Respond to happiness with positive excitement

        if (emotionStep == 0) {
            soundPlayer.playWave(pathToSounds + "excited1.wav");
            showImage(pathToImages + "excited.png");
            sleep(soundPause);
            emotionStep++; // Go to next step
        }
        else if (emotionStep == 1) {
            // Go Forwards
            goToNextStep = travel(0.2,FAST_MOVE,0,0);
            if (goToNextStep) emotionStep++; // Go to next step once done moving
        }
        else if (emotionStep == 2) {
            // Turn around
            goToNextStep = travel(0,0,M_PI * 2,FAST_SPIN);
            if (goToNextStep) emotionStep++;
        }
        else if (emotionStep == 3) {
            soundPlayer.playWave(pathToSounds + "excited2.wav");
            sleep(soundPause);
            clearEmotionState(); // Clear emotion reaction once done reaction (movement)
        }
        
    }
    else if (emotionValue == 4) {
        // Respond to sadness with anger

        if (emotionStep == 0) {
            soundPlayer.playWave(pathToSounds + "anger1.wav");
            showImage(pathToImages + "anger.png");
            sleep(soundPause);
            emotionStep++; // Go to next step
        }
        else if (emotionStep == 1) {
            // Go Forwards
            goToNextStep = travel(0.2,FAST_MOVE,0,0);
            if (goToNextStep) emotionStep++; // Go to next step once done moving
        }
        else if (emotionStep == 2) {
            soundPlayer.playWave(pathToSounds + "anger2.wav");
            sleep(soundPause);
            emotionStep++; // Go to next step
        }
        else if (emotionStep == 3) {
            // Go Forwards
            goToNextStep = travel(0.2,FAST_MOVE,0,0);
            if (goToNextStep) clearEmotionState(); // Clear emotion reaction once done reaction (movement)
        }
    }
    else if (emotionValue == 5) {
        // Respond to surprise with counter-surprise

        if (emotionStep == 0) {
            showImage(pathToImages + "surprise.png");
            emotionStep++; // Go to next step
        }
        else if (emotionStep == 1) {
            // Go backwards
            goToNextStep = travel(-0.3,-FAST_MOVE,0,0);
            if (goToNextStep) emotionStep++; // Go to next step once done moving
        }
        else if (emotionStep == 2) {
            soundPlayer.playWave(pathToSounds + "surprise1.wav");
            sleep(soundPause);
            emotionStep++; // Go to next step
        }
        else if (emotionStep == 3) {
            // Go Forwards
            goToNextStep = travel(0.2,SLOW_MOVE,0,0);
            if (goToNextStep) emotionStep++; // Go to next step once done moving
        }
        else if (emotionStep == 4) {
            soundPlayer.playWave(pathToSounds + "surprise2.wav");
            sleep(soundPause);
            clearEmotionState(); // Clear emotion reaction once done reaction (movement)
        }
        
    }
    else if (emotionValue == 6) {
        // Respond to neutral with pride
        if (emotionStep == 0) {
            soundPlayer.playWave(pathToSounds + "pride1.wav");
            showImage(pathToImages + "pride.png");
            sleep(soundPause);
            emotionStep++; // Go to next step
        }
        else if (emotionStep == 1) {
            // Spin around
            goToNextStep = travel(0,0,2*M_PI,SLOW_SPIN);
            if (goToNextStep) emotionStep++; // Go to next step once done moving
        }
        else if (emotionStep == 2) {
            soundPlayer.playWave(pathToSounds + "pride2.wav");
            sleep(soundPause);
            clearEmotionState(); // Clear emotion reaction once done reaction (movement)
        }
    }

    // Check if we're done processing emotions
    if (emotionValue < 0) {
        emotionStep = 0;
        victimsEncountered++; // Increment victim count once done interacting with them
    }
}


// Read in an image and show it on the screen
void showImage(std::string fileLocation) {
        
    try {
        cv::Mat img = cv::imread(fileLocation, cv::IMREAD_UNCHANGED);

        // Set transparent pixels to white
        for (int i = 0; i < img.size().width; i++) {
            for (int j = 0; j < img.size().width; j++) {

                // Is alpha less than half? (likely transparent)
                int index = 4 *i + j * 4 * img.size().width + 3;
                if (img.data[index] < 127 )  {
                    img.data[index] = 255;
                    img.data[index-1] = 255;
                    img.data[index-2] = 255;
                    img.data[index-3] = 255;
                }
            }
        }

        cv::resize(img, img,cv::Size(480,480)); // Resize to be visible on the screen

        cv::imshow("Current Response", img);
        cv::waitKey(50); // Wait 50 ms so image appears
    }
    catch (...) {
        ROS_ERROR("Failure to read image at following location:\n\t%s", fileLocation.c_str());
    }
    
}

// Act whenever we've detected an emotion present
// Currently just states and records the emotion
void emotionCallback(const std_msgs::Int32::ConstPtr& msg) {
    emotionValue = msg->data;

    ROS_INFO("Emotion %d detected (%s). Interacting with lifeform %d.", emotionValue, emotionName[emotionValue], victimsEncountered + 1);
}

// Returns most recent emotion scanned
int32_t readEmotion(void) {
    //emotionDetected = false; // Record the scan
    return emotionValue;
}


// Clear the emotion handler state
void clearEmotionState(void) {
    emotionValue = -1;
}

