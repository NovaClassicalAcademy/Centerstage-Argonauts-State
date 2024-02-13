package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PixelLifterB {

    int POSITION_FLOOR = 0;
    int POSITION_HOVER = 155;
    int POSITION_CARRY = 588;
    int POSITION_DUMP = 1639;
    double INTAKE_ACQUIRE_SPEED = .8;
    double INTAKE_REJECT_SPEED = -.4;

    enum POSITION{
        FLOOR,
        HOVER,
        CARRY,
        DUMP
    }

    double mPowerLimit = .1;
    DcMotor mLiftMotor;
    DcMotor mIntakeSpinner;

    public PixelLifterB(DcMotor liftMotor,
                        DcMotor intakeSpinner,
                        double powerLimit){

        mLiftMotor = liftMotor;
        mIntakeSpinner = intakeSpinner;
        mPowerLimit = powerLimit;

    }

    public void moveToFloor(){
            goToTarget(mLiftMotor, POSITION_FLOOR,mPowerLimit);
    }

    public void moveToHover(){
        goToTarget(mLiftMotor, POSITION_HOVER,mPowerLimit);

    }

    public void dumpPixel(){
        moveLifter(mLiftMotor, POSITION_DUMP,mPowerLimit);
    }

    public void moveCarry(){
        goToTarget(mLiftMotor, POSITION_CARRY,mPowerLimit);
    }

    public int getLifterCurrentPosition (){
        return mLiftMotor.getCurrentPosition();
    }

    public void acquirePixel(){
        //spin intake to suck in a pixel
        mIntakeSpinner.setPower(INTAKE_ACQUIRE_SPEED);
    }

    public void rejectPixel(){
        //spin intake to spit out a pixel
        mIntakeSpinner.setPower(INTAKE_REJECT_SPEED);
    }

    public void goToTarget(DcMotor motor,int targetPos,double powLimit) {
        /*

         * Proportional Integral Derivative Controller w/ Low pass filter and anti-windup
ADAPTED FROM: https://www.ctrlaltftc.com/the-pid-controller/practical-improvements-to-pid
Also: P: the further you are from where you want to be, the harder you should try to get there.

I: the longer you haven’t been where you want to be, the harder you should try to get there.

D: if you’re quickly getting close to where you want to be, slow down.

         */
//PID constants
        double Kp = .2;
        double Ki = .1;
        double Kd = .5;

        double reference = targetPos;
        double lastReference = reference;
        double integralSum = 0;

        double lastError = 0;

        double maxIntegralSum = .5;

        double a = 0.8; // a can be anything from 0 < a < 1
        double previousFilterEstimate = 0;
        double currentFilterEstimate = 0;
        double out = 0;
        double error = 0;
        double errorChange = 0;
        double derivative = 0;
        double encoderPosition = motor.getCurrentPosition();

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();
        encoderPosition = motor.getCurrentPosition();

        while (encoderPosition != targetPos) {
            // obtain the encoder position
            encoderPosition = motor.getCurrentPosition();
            // calculate the error
            error = reference - encoderPosition;

            errorChange = (error - lastError);

            // filter out high frequency noise to increase derivative performance
            currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
            previousFilterEstimate = currentFilterEstimate;

            // rate of change of the error
            derivative = currentFilterEstimate / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());


            // max out integral sum
            if (integralSum > maxIntegralSum) {
                integralSum = maxIntegralSum;
            }

            if (integralSum < -maxIntegralSum) {
                integralSum = -maxIntegralSum;
            }

            // reset integral sum upon setpoint changes
            if (reference != lastReference) {
                integralSum = 0;
            }

            out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            out = out * powLimit;
            motor.setPower(out);

            lastError = error;

            lastReference = reference;

            // reset the timer for next time
            timer.reset();

        }
    }
        public void moveLifter(DcMotor liftMotor,int targetPos,double powLimit){

            double Kp = .4;//proportional
            double Ki = .1;//integral
            double Kd = .1;//differential

            int reference = targetPos;

            double integralSum = 0;

            double lastError = 0;
            double encoderPosition = 0;
            double error = 0;
            double derivative = 0;
            double motorPower = 0;

// Elapsed timer class from SDK, please use it, it's epic
            ElapsedTime timer = new ElapsedTime();

           // while (liftMotor.getCurrentPosition() != reference) {
       if(liftMotor.getCurrentPosition() != reference){

                // obtain the encoder position
                encoderPosition = liftMotor.getCurrentPosition();
                // calculate the error
                error = reference - encoderPosition;

                // rate of change of the error
                derivative = (error - lastError) / timer.seconds();

                // sum of all error over time
                integralSum = integralSum + (error * timer.seconds());

                motorPower = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                liftMotor.setPower(motorPower);

                lastError = error;

                // reset the timer for next time
                timer.reset();
        }

    }
}
