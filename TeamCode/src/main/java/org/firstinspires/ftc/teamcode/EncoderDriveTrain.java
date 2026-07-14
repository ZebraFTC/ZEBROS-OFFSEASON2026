package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
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
    public DcMotor ExMotor2;
    public  DcMotor ExMotor1;
    public  DcMotor intake;
    public DcMotor bucket;
    public Servo leftServo;
    public Servo rightServo;
    public Servo latch;
    public Servo outtakeLift;
    public RevTouchSensor touchSensor;
    private boolean isExtended = false;
    int revolution = (int) 538/3;
    @Override
    public void runOpMode() {
        FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        FrontRight = hardwareMap.get(DcMotor.class, "FR");
        BackLeft = hardwareMap.get(DcMotor.class, "BL");
        BackRight = hardwareMap.get(DcMotor.class, "BR");
        ExMotor2 = hardwareMap.get(DcMotor.class, "RE");
        ExMotor1 = hardwareMap.get(DcMotor.class, "EX");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bucket = hardwareMap.get(DcMotor.class, "bucket");
        leftServo = hardwareMap.get(Servo.class, "LS");
        rightServo = hardwareMap.get(Servo.class, "RS");
        latch = hardwareMap.get(Servo.class, "latch");
        outtakeLift = hardwareMap.get(Servo.class, "OL");
        touchSensor = hardwareMap.get(RevTouchSensor.class, "touch");
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ExMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.REVERSE);

        ExMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ExMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ExMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucket.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            double drive = -0.5*gamepad1.left_stick_y;
            double strafe = -0.5*gamepad1.left_stick_x;
            double turn = 0.5*gamepad1.right_stick_x;
            FrontLeft.setPower(drive+turn+strafe);
            FrontRight.setPower(drive-turn+strafe);
            BackLeft.setPower(drive+turn-strafe);
            BackRight.setPower(drive-turn-strafe);

            if (gamepad1.left_bumper && isExtended == true) {
                retractProcess();
            }
            if (gamepad1.right_bumper && isExtended == false) {
                extendProcess();
            }

            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            if (gamepad1.dpad_up) {
                leftServo.setPosition(0.75);
                rightServo.setPosition(0.75);
                /***
                 servo lift, outtake, servo close
                 ***/
            }
            if (gamepad1.dpad_down) {
                leftServo.setPosition(0);
                rightServo.setPosition(0);
            }
        }
    }

    private void extendProcess() {
        /***
         rb = turn motor little, move servo to 0.16, normal extension code
         ***/
        bucket.setTargetPosition(bucket.getCurrentPosition() + (40));

        bucket.setPower(0.3);

        bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double timeout = getRuntime() + 5.0;

        if ((bucket.isBusy()  && getRuntime() < timeout)) {
            telemetry.addLine("bucket lifting");
        } else {
            bucket.setPower(0);
        }

        latch.setPosition(0.16);

        extension();
    }
    private void retractProcess() {
        /***
         lb = retract until button, servo turn, unwind a little, bucket down
         ***/
        retraction();
        bucket.setTargetPosition(bucket.getCurrentPosition() - (40));

        bucket.setPower(0.3);

        bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double timeout = getRuntime() + 5.0;

        if ((bucket.isBusy()  && getRuntime() < timeout)) {
            telemetry.addLine("bucket lifting");
        } else {
            bucket.setPower(0);
        }
    }
    private void extension() {
        ExMotor1.setTargetPosition(ExMotor1.getCurrentPosition() - ((int) 4.5*revolution));
        ExMotor2.setTargetPosition(ExMotor2.getCurrentPosition() - ((int) 4.5*revolution));

        ExMotor1.setPower(0.2);
        ExMotor2.setPower(0.2);

        ExMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if ((ExMotor1.isBusy()
                || ExMotor2.isBusy())) {

            telemetry.addLine("Extending");
        } else {
            ExMotor1.setPower(0);
            ExMotor2.setPower(0);
        }

        isExtended = true;
    }

    private void retraction() {
//        ExMotor1.setTargetPosition(ExMotor1.getCurrentPosition() - (revolution));
//        ExMotor2.setTargetPosition(ExMotor2.getCurrentPosition() - (revolution));

        ExMotor1.setPower(1);
        ExMotor2.setPower(1);

//        ExMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        ExMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        if ((ExMotor1.isBusy() || ExMotor2.isBusy())) {
//            telemetry.addLine("Retracting");
//        } else {
//            ExMotor1.setPower(0);
//            ExMotor2.setPower(0);
//        }

        if (touchSensor.isPressed()) {
            ExMotor1.setPower(0);
            ExMotor2.setPower(0);
            latch.setPosition(0);

            //unwind a little
            ExMotor1.setTargetPosition(ExMotor1.getCurrentPosition() + (90));
            ExMotor2.setTargetPosition(ExMotor2.getCurrentPosition() + (90));

            ExMotor1.setPower(0.1);
            ExMotor2.setPower(0.1);

            ExMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ExMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if ((ExMotor1.isBusy() || ExMotor2.isBusy())) {
                telemetry.addLine("Unwinding a little");
            } else {
                ExMotor1.setPower(0);
                ExMotor2.setPower(0);
            }
        }

        isExtended = false;
    }
}
