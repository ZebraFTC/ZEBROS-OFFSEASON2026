package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.Mechanisms.AdrcDriveTrain;
import org.firstinspires.ftc.teamcode.LimelightHandler;


@TeleOp
public class AdrcBasicDriveTrain extends LinearOpMode {
    public DcMotor intake;
    public DcMotor transfer;
    public DcMotor shooter;
    public Servo ramp;

    @Override
    public void runOpMode() {
        //LimelightHandler.setupLimelight(hardwareMap, "limelight", 0);
        waitForStart();
        while (opModeIsActive()) {
            intake = hardwareMap.get(DcMotor.class, "Intake");
            transfer = hardwareMap.get(DcMotor.class, "Transfer");
            shooter = hardwareMap.get(DcMotor.class, "Shooter");
            ramp = hardwareMap.get(Servo.class, "Ramp");

            shooter.setDirection(DcMotorSimple.Direction.REVERSE);
            transfer.setDirection(DcMotorSimple.Direction.REVERSE);

            // get info from limeLight cam
            //LLResult result = LimelightHandler.grabInformation();
            /*
            if (result != null && result.isValid()) {
                java.util.List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();

                //get april tags
                for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult tag : aprilTags) {

                    int tagID = tag.getFiducialId();
                    double tagX = tag.getTargetXDegrees();
                    double tagY = tag.getTargetYDegrees();
                    double tagArea = tag.getTargetArea();

                    //print stuff
                    telemetry.addData("AprilTag", "ID: %d", tagID);
                    telemetry.addData(" -> Tag X Offset", "%.2f°", tagX);
                    telemetry.addData(" -> Tag Y Offset", "%.2f°", tagY);
                }
            }
        }
             */
            AdrcDriveTrain driveTrain = new AdrcDriveTrain(
                    hardwareMap.get(DcMotorEx.class, "FL"),
                    hardwareMap.get(DcMotorEx.class, "FR"),
                    hardwareMap.get(DcMotorEx.class, "BL"),
                    hardwareMap.get(DcMotorEx.class, "BR"));
            double WC = AdrcDriveTrain.WC;
            double WO = AdrcDriveTrain.WO;

            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            driveTrain.drive(drive, strafe, turn);

            if (gamepad1.right_trigger_pressed) {
                intake.setPower(.8);
                transfer.setPower(.8);
            } else if (gamepad1.left_trigger_pressed) {
                intake.setPower(-.8);
                transfer.setPower(-.8);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
            }
            if (gamepad1.dpad_up) {
                shooter.setPower(.5);
            }
            if (gamepad1.dpad_down) {
                shooter.setPower(0);
            }
            if (gamepad1.a) {
                ramp.setPosition(0);
            }
            if (gamepad1.b) {
                ramp.setPosition(.8);
            }
        }
    }
}


