package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain extends Mechanism{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    private static final double MAX_SPEED = 0.8;
    public DriveTrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
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
}
