package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PixelLifterB {

    int POSITION_FLOOR = 0;
    int POSITION_HOVER = 155;
    int POSITION_CARRY = 588;
    int POSITION_DUMP = 1600;//was 1639
    double INTAKE_ACQUIRE_SPEED = .8;
    double INTAKE_REJECT_SPEED = -.4;

    enum POSITION {
        FLOOR,
        HOVER,
        CARRY,
        DUMP
    }

    double mPowerLimit = .9;
    DcMotorEx mLiftMotor;
    DcMotor mIntakeSpinner;

    public PixelLifterB(DcMotorEx liftMotor,
                        DcMotor intakeSpinner,
                        double powerLimit) {

        mLiftMotor = liftMotor;
        mIntakeSpinner = intakeSpinner;
        mPowerLimit = powerLimit;
        mLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        mLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void moveToFloor() {
        goToTarget(mLiftMotor, POSITION_FLOOR, 700);

    }

    public void moveToHover() {
        goToTarget(mLiftMotor, POSITION_HOVER, 700);

    }

    public void dumpPixel() {
        goToTarget(mLiftMotor, POSITION_DUMP, 1200);
    }

    public void moveCarry() {
        goToTarget(mLiftMotor, POSITION_CARRY, 600);
    }

    public void resetEncoder(){
        mLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getLifterCurrentPosition() {
        return mLiftMotor.getCurrentPosition();
    }

    public void acquirePixel() {
        //spin intake to suck in a pixel
        mIntakeSpinner.setPower(INTAKE_ACQUIRE_SPEED);
    }

    public void rejectPixel() {
        //spin intake to spit out a pixel
        mIntakeSpinner.setPower(INTAKE_REJECT_SPEED);
    }


    public void goToTarget(DcMotorEx motor,int targetPos,double velocity) {
        motor.setTargetPosition(targetPos);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setVelocity(velocity);
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
