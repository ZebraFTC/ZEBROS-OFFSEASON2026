package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.Mechanisms.Slides;

@TeleOp
public class OpMode extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public  DcMotor leftMotor;
    public DcMotor rightMotor;
    public  DcMotor intake;
    public DcMotor bucket;
    public Servo leftServo;
    public Servo rightServo;
    public Servo latch;
    public Servo outtakeLift;
    public RevTouchSensor touchSensor;
    private boolean touchWasPressed;
    private boolean latchClosed;
    int revolution = 145;
    private double timer;
    @Override
    public void runOpMode() {
        latchClosed = true;
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        leftMotor = hardwareMap.get(DcMotor.class, "EX");
        rightMotor = hardwareMap.get(DcMotor.class, "RE");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bucket = hardwareMap.get(DcMotor.class, "bucket");
        leftServo = hardwareMap.get(Servo.class, "LS");
        rightServo = hardwareMap.get(Servo.class, "RS");
        latch = hardwareMap.get(Servo.class, "latch");
        outtakeLift = hardwareMap.get(Servo.class, "OL");
        touchSensor = hardwareMap.get(RevTouchSensor.class, "touch");
        int bucketPosition = bucket.getCurrentPosition();

        DriveTrain driveTrain = new DriveTrain(frontLeft, frontRight, backLeft, backRight);
        Slides slides = new Slides(leftMotor, rightMotor, touchSensor, latch);
        Intake intakeClass = new Intake(intake, leftServo, rightServo);
        Outtake outtake = new Outtake(bucket, outtakeLift);

        waitForStart();
        while (opModeIsActive()) {
            double driveForward = -0.5*gamepad1.left_stick_y;
            double strafeRight = -0.5*gamepad1.left_stick_x;
            double turnCW = 0.5*gamepad1.right_stick_x;
            driveTrain.drive(driveForward, strafeRight, turnCW);

            telemetry.addData("touch sensor", touchSensor.isPressed());
            telemetry.addData("runtime",getRuntime());
            telemetry.addData("latch position", latch.getPosition());
            telemetry.addData("Left Motor", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor", rightMotor.getCurrentPosition());

            if (gamepad1.left_bumper) {
                slides.retract();
            }
            if (gamepad1.right_bumper) {
                slides.extend();
            }

            double intakePower = gamepad1.right_trigger-gamepad1.left_trigger;
            intakeClass.runIntake(intakePower);

            if (gamepad1.dpad_up) {
                outtake.moveLid("open");
                intakeClass.runIntake(-1);
            } else if (gamepad1.dpad_down) {
                outtake.moveLid("close");
                intakeClass.runIntake(0);
            }

            if (gamepad1.dpad_left) {
                intakeClass.flipTransfer("up");
            } else if (gamepad1.dpad_right) {
                intakeClass.flipTransfer("down");
            }

            if (gamepad1.a) {
                outtake.moveLid("open");
            } else if (gamepad1.b) {
                outtake.moveLid("close");
            }

            if (gamepad1.y) {
                outtake.moveBucket("up", bucketPosition);
            } else if (gamepad1.x) {
                outtake.moveBucket("down", bucketPosition);
            }

            telemetry.update();
            touchWasPressed = touchSensor.isPressed();
        }
    }
}
