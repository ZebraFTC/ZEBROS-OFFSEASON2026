package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Slides extends Mechanism{
    DcMotor leftMotor;
    DcMotor rightMotor;
    RevTouchSensor touchSensor;
    Servo latch;
    private boolean touchWasPressed;
    private boolean latchClosed;
    int revolution = 145;
    private double timer;
    public Slides(DcMotor leftMotor, DcMotor rightMotor, RevTouchSensor touchSensor, Servo latch) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.touchSensor = touchSensor;
        this.latch = latch;
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void extend() {
        latch.setPosition(0);
        latchClosed = false;
        //intake servos

        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() - ((int) 4.7*revolution));
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() - ((int) 4.7*revolution));

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void retract() {
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(1);
        rightMotor.setPower(1);
    }

    private void closeLatch(double time) {
        latch.setPosition(0.16);
        latchClosed = true;
        timer = time + 0.5;
    }

    private boolean isLatchClosed(double time) {
        if (time >= timer && latchClosed) {
            return true;
        }
        return false;
    }

    @Override
    public void update(double time) {
        if (touchSensor.isPressed() && !touchWasPressed) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            closeLatch(time);
        }
        if (isLatchClosed(time)) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }


    }
}
