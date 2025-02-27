#pragma once
#include "main.h"

class Timer {
    private:
        uint32_t startTime;
        uint32_t currentTime;
        uint32_t endTime;
        uint32_t pauseTime;
        uint32_t timeDuration;
        bool isRunning;
    public:
        Timer(uint32_t time) {
            startTime = 0;
            endTime = 0;
            pauseTime = 0;
            timeDuration = time;
            isRunning = false;
        }

        void setTime(uint32_t time) {
            timeDuration = time;
        }

        void start() {
            if (isRunning) return;
            if (timeDuration == 0) return;
            startTime = pros::millis();
            endTime = startTime + timeDuration;
            isRunning = true;
        }

        void pause() {
            if (!isRunning) return;
            pauseTime = pros::millis();
            isRunning = false;
        }

        void resume() {
            if (isRunning) return;
            if (timeDuration == 0) return;
            endTime += (pros::millis() - pauseTime);
            isRunning = true;
        }

        bool isDone() {
            if (timeDuration == 0) return false;
            if (!isRunning) return false;
            return pros::millis() >= endTime;
        }

        bool isTimerRunning() {
            if (isDone()) {
                isRunning = false;
            }
            return isRunning;
        }

        uint32_t getTimeRemaining() {
            if (!isRunning) return 0;
            return endTime - pros::millis();
        }
        
        void reset() {
            startTime = 0;
            endTime = 0;
            isRunning = false;
            pauseTime = 0;
        }
};
