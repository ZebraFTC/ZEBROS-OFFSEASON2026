package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake extends Mechanism{
    public DcMotor bucket;
    public Servo outtakeLift;
    public static final int THREE_TWELVE_MOTOR_REV = 538;
    public Outtake(DcMotor bucket, Servo outtakeLift) {
        this.bucket = bucket;
        this.outtakeLift = outtakeLift;

        bucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucket.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int bucketPosition = bucket.getCurrentPosition();
    }

    //valid parameters: up,down
    public void moveBucket(String direction, int bucketPosition) {
        if (direction .equals("up")) {
            outtakeLift.setPosition(0);
            bucket.setTargetPosition(bucketPosition + ((int) (0.5* THREE_TWELVE_MOTOR_REV)));

            bucket.setPower(0.2);

            bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (direction .equals("down")) {
            bucket.setTargetPosition(bucketPosition);

            bucket.setPower(0.2);

            bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeLift.setPosition(0.1);
        }
    }

    //valid parameters: open,closed
    public void moveLid(String status) {
        if (status .equals("open")) {
            outtakeLift.setPosition(0.1);
        } else if (status .equals("closed")) {
            outtakeLift.setPosition(0);
        }
    }

    @Override
    public void update(double time) {
        //update later
    }
}
