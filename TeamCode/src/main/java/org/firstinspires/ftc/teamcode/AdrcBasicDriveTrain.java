package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Mechanisms.AdrcDriveTrain;

@TeleOp
public class AdrcBasicDriveTrain extends LinearOpMode {
    @Override
    public void runOpMode() {
        AdrcDriveTrain driveTrain = new AdrcDriveTrain(
                hardwareMap.get(DcMotorEx.class, "FL"),
                hardwareMap.get(DcMotorEx.class, "FR"),
                hardwareMap.get(DcMotorEx.class, "BL"),
                hardwareMap.get(DcMotorEx.class, "BR"));

        waitForStart();
        while (opModeIsActive()) {
            double drive  = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;   // kept from your original code
            double turn   =  gamepad1.right_stick_x;
            driveTrain.drive(drive, strafe, turn);
        }
    }
}