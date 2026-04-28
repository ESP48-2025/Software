#ifndef BUGGY_H
#define BUGGY_H

#include "mbed.h"
#include "Motor.h"
#include "PID.h"
#include "LineSensor.h"

class Buggy {
private:
    Motor &left;
    Motor &right;

    PID &pidL;
    PID &pidR;
    PID &steerPID;

    LineSensor &sensors;

    Ticker controlTicker;

    float baseSpeed;

    float filtL;
    float filtR;

    float pwmL, pwmR;

    float currentTarget;

    bool isRunning;
    bool lineLostFlag;
    
    bool lineLostBuffer;    // 

    volatile bool inTurn;   // 

public:

    Buggy(Motor &l, Motor &r,
          PID &pL, PID &pR,
          PID &steer,
          LineSensor &sens,
          float baseSpeed)
        : left(l), right(r),
          pidL(pL), pidR(pR),
          steerPID(steer),
          sensors(sens),
          baseSpeed(baseSpeed)
    {
        filtL = 0.0f;
        filtR = 0.0f;
        currentTarget = 0.0f;

        isRunning = false;
        lineLostFlag = false;
        inTurn = false;
    }

    // ===== START =====
    void start() {
        left.begin();
        right.begin();
        sensors.start();

        wait_ms(150);

        filtL = 0.0f;
        filtR = 0.0f;
        currentTarget = 0.0f;

        lineLostFlag = false;
        isRunning = true;
        inTurn = false;

        controlTicker.attach(callback(this, &Buggy::controlLoop), 0.01f);
    }

    // ===== CONTROL LOOP =====
    void controlLoop() {
        if (!isRunning || inTurn) return;   // <<< STOP CONTROL DURING TURN

        float speedL = -left.readSpeedMs();
        float speedR =  right.readSpeedMs();

        float alphaL = (fabs(filtL) < 0.06f) ? 0.3f : 0.15f;
        float alphaR = (fabs(filtR) < 0.06f) ? 0.3f : 0.15f;

        filtL = alphaL * speedL + (1.0f - alphaL) * filtL;
        filtR = alphaR * speedR + (1.0f - alphaR) * filtR;

        if (sensors.isLineLost()) {
            // if (lineLostDistance == 0){
            //     lineLostTime.start();
            // }
            // lineLostDistance = (speedL + speedR)/2.0f * lineLostTime.read_ms();
            // // average speed * time in lost line = lost distance
            // if (lineLostDistance > 7){
            //     lineLostTime.stop();
            //     lineLostTime.reset();
            //     lineLostDistance = 0;
            //     // stop
            //     left.setSpeed(0.0f);
            //     right.setSpeed(0.0f);
            //     lineLostFlag = true;
            // }
            // stop
            left.setSpeed(0.0f);
            right.setSpeed(0.0f);
            lineLostFlag = true;
            return;
        }

        float position = sensors.getPosition();
        float steer = steerPID.update(0.0f, position);

        if (currentTarget < baseSpeed) {
            currentTarget += 0.001f;
            if (currentTarget > baseSpeed)
                currentTarget = baseSpeed;
        }

        float targetL = currentTarget + steer;
        float targetR = currentTarget - steer;

        float ctrlL = pidL.update(targetL, filtL);
        float ctrlR = pidR.update(targetR, filtR);

        float kf = 0.5f;

        pwmL = kf * targetL + ctrlL;
        pwmR = kf * targetR + ctrlR;

        if (pwmL > 0.95f) pwmL = 0.95f;
        if (pwmL < 0.05f) pwmL = 0.05f;

        if (pwmR > 0.95f) pwmR = 0.95f;
        if (pwmR < 0.05f) pwmR = 0.05f;

        left.setSpeed(pwmL);
        right.setSpeed(pwmR);
    }

    float rtnpwmL(void){
        return pwmL;
    }
    float rtnpwmR(void){
        return pwmR;
    }

    // ===== STOP =====
    void stop() {
        isRunning = false;
        inTurn = false;

        controlTicker.detach();

        left.setSpeed(0.0f);
        right.setSpeed(0.0f);

        pidL.reset();
        pidR.reset();
        steerPID.reset();

        currentTarget = 0.0f;
    }

    // ===== SAFE UPDATE =====
    void update() {
        if (lineLostFlag && isRunning && !inTurn) {
            stop();
        }
    }

    // ===== FIXED TURNAROUND =====
    void turnaround() {

        if (inTurn) return;   // <<< PREVENT DOUBLE ENTRY
        inTurn = true;

        controlTicker.detach(); // stop PID loop fully

        // clear motion
        left.setSpeed(0.0f);
        right.setSpeed(0.0f);
        wait_ms(150);

        // pivot turn (simple differential)
        left.setDirection(1);
        right.setDirection(0);

        left.setSpeed(0.3f);
        right.setSpeed(0.3f);

        wait_ms(1150);

        left.setSpeed(0.0f);
        right.setSpeed(0.0f);

        wait_ms(150);

        // restore direction
        left.setDirection(1);
        right.setDirection(1);

        // reset flags
        lineLostFlag = false;
        inTurn = false;

        // restart controller
        isRunning = true;
        controlTicker.attach(callback(this, &Buggy::controlLoop), 0.01f);
    }

    bool shouldStop() {
        return lineLostFlag;
    }

    void setBaseSpeed(float speed) {
        baseSpeed = speed;
    }
};

#endif
