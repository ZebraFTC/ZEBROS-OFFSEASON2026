package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.DriveTrain;


@Autonomous
public class Auto extends OpMode {
    public static final int DRIVE_TIMEOUT = 8;
    DriveTrain drive;
    ElapsedTime stepTimer;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    public enum Steps {

        LIFT_ARM,

        DRIVE_TO_PARK,

        DROP_ARM,
    }

    Steps currentStep;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        stepTimer = new ElapsedTime();
        drive = new DriveTrain(frontLeft,frontRight,backLeft,backRight);
        currentStep = Steps.DRIVE_TO_PARK;
    }


    @Override
    public void loop() {
        switch (currentStep) {
            case DRIVE_TO_PARK:
                if(drive.goToPoint(0, 800) || stepTimer.time() > DRIVE_TIMEOUT){
                    currentStep = Steps.DROP_ARM;
                    stepTimer.reset();
                }
                break;
            case DROP_ARM:
        }
    }
}
