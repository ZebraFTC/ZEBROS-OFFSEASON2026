package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class EncoderDriveTrain extends LinearOpMode {
    public double MovePower;

    public double ShootSpeed;
    public DcMotorEx FrontLeft;
    public DcMotorEx FrontRight;
    public DcMotorEx BackLeft;
    public DcMotorEx BackRight;
    public Servo flap;
    public DcMotor intake;
    public DcMotor transfer;
    public DcMotorEx shooter;
    public Servo ramp;


    @Override
    public void runOpMode() {
        MovePower = .95;
        ShootSpeed = 0.25;
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        ramp = hardwareMap.get(Servo.class, "Ramp");
        flap = hardwareMap.get(Servo.class, "Flap");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        BackLeft = hardwareMap.get(DcMotorEx.class, "BL");
        BackRight = hardwareMap.get(DcMotorEx.class, "BR");

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ramp.setPosition(.8);

        waitForStart();
        while (opModeIsActive()) {

            double drive = -0.8 * gamepad1.left_stick_y;
            double strafe = -1.7 * gamepad1.left_stick_x;
            double turn = 0.8 * gamepad1.right_stick_x;
            float FLPower = (float) (drive + turn + strafe);
            float FRPower = (float) (drive - turn - strafe);
            float BLPower = (float) (drive + turn - strafe);
            float BRPower = (float) (drive - turn + strafe);

            FrontLeft.setPower(FLPower * MovePower);
            FrontRight.setPower(FRPower * MovePower);
            BackLeft.setPower(BLPower * MovePower);
            BackRight.setPower(BRPower * MovePower);


            if (gamepad2.right_trigger_pressed) {
                intake.setPower(1);
                transfer.setPower(1);
            } else if (gamepad2.left_trigger_pressed) {
                intake.setPower(-.8);
                transfer.setPower(-.8);
            } else if (gamepad2.y) {
                intake.setPower(.25);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
            }

            if (gamepad2.right_bumper) {
                shooter.setPower(ShootSpeed);
            }

            if (gamepad2.left_bumper) {
                shooter.setPower(0);
            }
            if (gamepad2.dpad_right) {
                ShootSpeed = .3;
            }
            if (gamepad2.dpad_left) {
                ShootSpeed = .35;
            }
            if (gamepad2.dpad_up) {
                ShootSpeed = .4;
            }
            if (gamepad2.dpad_down) {
                ShootSpeed = .15;
            }
            if (gamepad2.a) {
                flap.setPosition(0);
            }
            if (gamepad2.b) {
                flap.setPosition(.3);
            }
            if (gamepad1.a) {
                ramp.setPosition(0);
            }
            if (gamepad1.b) {
                ramp.setPosition(.8);
            }
            if (gamepad1.dpad_up) {
                MovePower = 1.25;
            }
            if (gamepad1.dpad_left) {
                MovePower = .95;
            }
            if (gamepad1.dpad_down){
                MovePower = .7;
            }
            telemetry.addData("Shoot Power",String.valueOf(ShootSpeed));
            telemetry.addData("Move Power",String.valueOf(MovePower));
            telemetry.update();

        }
    }
}
