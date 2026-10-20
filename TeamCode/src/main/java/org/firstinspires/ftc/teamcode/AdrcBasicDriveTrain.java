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
        double WC = AdrcDriveTrain.WC;
        double WO = AdrcDriveTrain.WO;
        waitForStart();
        while (opModeIsActive()) {
            //Tuning code
            //use y and b for WC and x and a for WO
            /*
            if (gamepad1.y) {
                WC += 0.5;
                sleep(200);
            }
            if (gamepad1.b) {
                WC -= 0.5;
                sleep(200);
            }
            if (gamepad1.x) {
                WO += 0.5;
                sleep(200);
            }
            if (gamepad1.a) {
                WO -= 0.5;
                sleep(200);
            }

            telemetry.addData("WC value", WC);
            telemetry.addData("WO value", WO);
            telemetry.update();

             */
            double drive  = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;
            driveTrain.drive(drive, strafe, turn);
        }
    }
}
