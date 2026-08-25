package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveTrain extends Mechanism{
    private static final double DRIVE_TO_POWER = 0.5;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    boolean firstTime = true;
    private double xPos;
    private double yPos;
    private static final double MAX_SPEED = 0.8;
    public DriveTrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        yPos = 0;
    }

    public void drive(double driveForward, double strafeRight, double turnCW) {
        driveForward *= MAX_SPEED;
        strafeRight *= MAX_SPEED;
        turnCW *= MAX_SPEED;
        frontLeft.setPower(driveForward+turnCW+strafeRight);
        frontRight.setPower(driveForward-turnCW+strafeRight);
        backLeft.setPower(driveForward+turnCW-strafeRight);
        backRight.setPower(driveForward-turnCW-strafeRight);
    }
    @Override
    public void update(double time) {}

    public void setPosition(double x, double y) {
        xPos = x;
        yPos = y;
    }

    public boolean goToPoint(int xChange, int yChange) {
        if (firstTime) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            firstTime = false;
        }

        int frontLeftDesired = (yChange + xChange);
        int frontRightDesired = (yChange + xChange);
        int backLeftDesired = (yChange - xChange);
        int backRightDesired = (yChange - xChange);

        frontLeft.setTargetPosition(frontLeftDesired);
        frontRight.setTargetPosition(frontRightDesired);
        backLeft.setTargetPosition(backLeftDesired);
        backRight.setTargetPosition(backRightDesired);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(DRIVE_TO_POWER);
        frontRight.setPower(DRIVE_TO_POWER);
        backRight.setPower(DRIVE_TO_POWER);
        backLeft.setPower(DRIVE_TO_POWER);

        if (!frontLeft.isBusy() && !frontRight.isBusy() && !backLeft.isBusy() && !backRight.isBusy()){
            firstTime = true;
            return true;
        }
        return false;
}
}
