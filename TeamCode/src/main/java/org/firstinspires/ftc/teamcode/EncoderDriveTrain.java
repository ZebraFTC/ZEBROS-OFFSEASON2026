package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Mechanisms.ADRC_driver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class EncoderDriveTrain extends LinearOpMode {
    public double MovePower;
    boolean shoot;
    public double targetShootRPM;
    public DcMotorEx FrontLeft, FrontRight, BackLeft, BackRight;
    public Servo flap, ramp;
    public DcMotor intake, transfer;
    public DcMotorEx shooter;

    @Override
    public void runOpMode() {
        shoot = false;
        MovePower = .95;
        targetShootRPM = 1000.0; // Default ticks/sec

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
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ramp.setPosition(.8);

        ADRC_driver shooter_adrc = new ADRC_driver(
                4000.0, // b0: tune based on flywheel acceleration
                40.0,   // omegaO: observer bandwidth
                10.0    // omegaC: controller bandwidth
        );

        waitForStart();

        // Initialize timer and observer AFTER waitForStart()
        long lastTimeNanos = System.nanoTime();
        shooter_adrc.reset(shooter.getVelocity());

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = (now - lastTimeNanos) / 1e9;
            lastTimeNanos = now;

            // --- Drivetrain ---
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

            // --- ADRC Shooter Velocity Control ---
            double currentVelocity = shooter.getVelocity();
            if (shoot) {
                double motorPower = shooter_adrc.calculate((targetShootRPM/60.0)*28.0, currentVelocity, dt);
                shooter.setPower(motorPower);
            } else {
                shooter.setPower(0);
                shooter_adrc.reset(currentVelocity); // Prevent windup while off
            }

            // --- Intake & Transfer (Fixed trigger syntax) ---
            if (gamepad2.right_trigger > 0.5) {
                intake.setPower(1);
                transfer.setPower(1);
            } else if (gamepad2.left_trigger > 0.5) {
                intake.setPower(-.8);
                transfer.setPower(-.8);
            } else if (gamepad2.y) {
                intake.setPower(.25);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
            }

            // --- Shooter Toggle & Speed Presets (in ticks/sec) ---
            if (gamepad2.right_bumper) shoot = true;
            if (gamepad2.left_bumper) shoot = false;

            if (gamepad2.dpad_down)  targetShootRPM = 1000.0;
            if (gamepad2.dpad_right) targetShootRPM = 2300.0;
            if (gamepad2.dpad_left)  targetShootRPM = 3000.0;
            if (gamepad2.dpad_up)    targetShootRPM = 4000.0;

            // --- Servos & Drive Speed ---
            if (gamepad2.a) flap.setPosition(0);
            if (gamepad2.b) flap.setPosition(.3);
            if (gamepad1.a) ramp.setPosition(0);
            if (gamepad1.b) ramp.setPosition(.8);

            if (gamepad1.dpad_up) MovePower = 1.25;
            if (gamepad1.dpad_left) MovePower = .95;
            if (gamepad1.dpad_down) MovePower = .7;

            telemetry.addData("Shooter Active", shoot);
            telemetry.addData("Target vs Actual Vel", "%.0f / %.0f", targetShootRPM, currentVelocity);
            telemetry.addData("Move Power", MovePower);
            telemetry.update();
        }
    }
}
