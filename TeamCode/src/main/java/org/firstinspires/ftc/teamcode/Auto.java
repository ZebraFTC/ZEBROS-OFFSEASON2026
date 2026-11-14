package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {
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

        if (opModeIsActive()) {
            FrontLeft.setPower(.3);
            FrontRight.setPower(.3);
            BackLeft.setPower(.3);
            BackRight.setPower(.3);

            sleep(1000);

            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }

    }
}

