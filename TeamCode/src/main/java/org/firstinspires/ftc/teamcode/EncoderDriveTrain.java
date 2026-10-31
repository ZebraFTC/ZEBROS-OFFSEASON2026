package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class EncoderDriveTrain extends LinearOpMode {
    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor BackLeft;
    public DcMotor BackRight;
    public DcMotor intake;
    public DcMotor transfer;
    public DcMotor shooter;
    public Servo ramp;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        ramp = hardwareMap.get(Servo.class, "Ramp");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        FrontRight = hardwareMap.get(DcMotor.class, "FR");
        BackLeft = hardwareMap.get(DcMotor.class, "BL");
        BackRight = hardwareMap.get(DcMotor.class, "BR");

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {

            double drive = -0.8 * gamepad1.left_stick_y;
            double strafe = -0.8 * gamepad1.left_stick_x;
            double turn = 0.8 * gamepad1.right_stick_x;
            FrontLeft.setPower(drive + turn + strafe);
            FrontRight.setPower(drive - turn - strafe);
            BackLeft.setPower(drive + turn - strafe);
            BackRight.setPower(drive - turn + strafe);


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