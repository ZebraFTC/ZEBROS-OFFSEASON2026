package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.security.cert.Extension;

@TeleOp
public class DriveTrain extends LinearOpMode {
    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor BackLeft;
    public DcMotor BackRight;
    public DcMotor Retraction;
    public  DcMotor Extension;
    @Override
    public void runOpMode() {
        FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        FrontRight = hardwareMap.get(DcMotor.class, "FR");
        BackLeft = hardwareMap.get(DcMotor.class, "BL");
        BackRight = hardwareMap.get(DcMotor.class, "BR");
        Retraction = hardwareMap.get(DcMotor.class, "RE");
        Extension = hardwareMap.get(DcMotor.class, "EX");
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            double drive = -0.5*gamepad1.left_stick_y;
            double strafe = -0.5*gamepad1.left_stick_x;
            double turn = 0.5*gamepad1.right_stick_x;
            FrontLeft.setPower(drive+turn-strafe);
            FrontRight.setPower(drive-turn+strafe);
            BackLeft.setPower(drive+turn+strafe);
            BackRight.setPower(drive-turn-strafe);

            double extend = gamepad1.left_trigger-gamepad1.right_trigger;

            if (extend > 0.5) {
                Extension.setPower(0.2);
                Retraction.setPower(0.2);
            }
            if (extend < -0.5) {
                Extension.setPower(-0.2);
                Retraction.setPower(-0.2);
            }

        }
    }
}