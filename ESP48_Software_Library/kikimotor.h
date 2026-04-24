#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"
#include "QEI.h"

#define tick_interval 0.01f


class Motor {
    private:
        //declare the motPWM
        // init stuff
        PwmOut motPWM;
        float speed;
        float encSpeed;

        DigitalOut isBipolar;
        DigitalOut wDirection;
        
        QEI encoder; // QEI cookbook - search it up - quadrature encoder class
        Ticker sampler;
        volatile float speed_ms;
        

    public:
            volatile int prev_pulse; // just text me i cba to explain this here

        Motor(
            PinName pwm, 
            
            PinName EncPIN0, 
            PinName EncPIN1, 
            
            PinName isNC, 
            int pulsePRev,
            
            PinName bipolar,
            PinName direction):
                motPWM(pwm),
                encoder(EncPIN0, EncPIN1, isNC, pulsePRev),
                isBipolar(bipolar),
                wDirection(direction)
                {
                    isBipolar = 0;
                    wDirection = 1;
                    prev_pulse = 0;
                    // pc.printf("Ticker started\r\n");
                }

        void setSpeed(float speed) {
            // pc.printf("Speed has been set.\r\n");
            motPWM.period_us(50);
            motPWM.write(1-speed);
            return;
        }

        void begin() {
            sampler.attach(callback(this, &Motor::readSpeed), tick_interval);
        }

        float readSpeedMs () {

            return speed_ms;
        }

        void setDirection (bool dir) {
            wDirection = dir;
            return;
        }

        void readSpeed() {
            // pc.printf("Success");
            // pc.printf("Pulses for motA is: %i\r\n", encoder.getPulses());
            int current_pulse = encoder.getPulses();
            float tick_rate = float(current_pulse - prev_pulse)/ tick_interval;
            prev_pulse = current_pulse;
            
            float rev_per_sec = tick_rate / ((float) 512 * 10.84);
            speed_ms = rev_per_sec * 0.075 * 3.1415962;
            
            //pc.printf("Speed is: %.2f\r\n", ticker_speed);
            // pc.printf("Wheel speed is: %f\r\n", wheel_speed);
            return;
        }
};

#endif
