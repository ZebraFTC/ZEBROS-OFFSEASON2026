package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class EncoderDriveTrain extends LinearOpMode {
    public static final int THREE_TWELVE_MOTOR_REV = 538;
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
    private boolean touchWasPressed;
    private boolean latchClosed;
    int revolution = 145;

    private double timer;
    @Override
    public void runOpMode() {
        latchClosed = true;
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

        int bucketPosition = bucket.getCurrentPosition();
        telemetry.addData("bucket", bucketPosition);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            double drive = -0.5*gamepad1.left_stick_y;
            double strafe = -0.5*gamepad1.left_stick_x;
            double turn = 0.5*gamepad1.right_stick_x;
            FrontLeft.setPower(drive+turn+strafe);
            FrontRight.setPower(drive-turn+strafe);
            BackLeft.setPower(drive+turn-strafe);
            BackRight.setPower(drive-turn-strafe);

            telemetry.addData("touch sensor", touchSensor.isPressed());
            telemetry.addData("runtime",getRuntime());
            telemetry.addData("latch position", latch.getPosition());


            if (gamepad1.left_bumper) {
                ExMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ExMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ExMotor1.setPower(1);
                ExMotor2.setPower(1);
            }

            if(touchSensor.isPressed() && !touchWasPressed){
                latch.setPosition(0.16);
                latchClosed = true;
                timer = getRuntime() + 0.5;
            }

            if (latchClosed)
            {
                telemetry.addData("they both", "work");
            }

            if (getRuntime() >= timer && latchClosed)
            {
                telemetry.addData("if this doesnt print", "smthing is wrong");
                ExMotor1.setPower(0);
                ExMotor2.setPower(0);
                //retractProcess();
            }



            if (gamepad1.right_bumper) {
                extendProcess();
            }

            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            if (gamepad1.dpad_up) {
                leftServo.setPosition(0);
                rightServo.setPosition(0);
                /***
                 servo lift, outtake, servo close
                 ***/
                timer = getRuntime() + 5.0;
                if (timer - getRuntime() <= 0) {
                    intake.setPower(0);
                    outtakeLift.setPosition(0.05);
                } else if (timer - getRuntime() <= 2.5) {
                    intake.setPower(-1);
                    outtakeLift.setPosition(0);
                } else {
                      telemetry.addData("waiting", "for timer");
                }
            }
            if (gamepad1.dpad_down) {
                leftServo.setPosition(0.7);
                rightServo.setPosition(0.7);
            }
            if (gamepad1.b){
                outtakeLift.setPosition(0);
            }
            else if (gamepad1.a) {
                outtakeLift.setPosition(0.08);
            }
            if (gamepad1.y) {
                bucket.setTargetPosition(bucketPosition + ((int) (0.5* THREE_TWELVE_MOTOR_REV)));

                bucket.setPower(0.5);

                bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad1.x) {
                bucket.setTargetPosition(bucketPosition);

                bucket.setPower(0.5);

                bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            telemetry.update();
            touchWasPressed = touchSensor.isPressed();
        }
    }

    private void extendProcess() {
        /***
         rb = turn motor little, move servo to 0.16, normal extension code
         ***/

        latch.setPosition(0);
        latchClosed = false;

        extension();
    }
    private void retractProcess() {
        /***
         lb = retract until button, servo turn, unwind a little, bucket down
         ***/
        //retraction();
        bucket.setTargetPosition(bucket.getCurrentPosition() - ((int) (0.75*revolution)));

        bucket.setPower(0.3);

        bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void extension() {
        ExMotor1.setTargetPosition(ExMotor1.getCurrentPosition() - ((int) 4.6*revolution));
        ExMotor2.setTargetPosition(ExMotor2.getCurrentPosition() - ((int) 4.6*revolution));

        ExMotor1.setPower(0.5);
        ExMotor2.setPower(0.5);

        ExMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        isExtended = true;
    }

    private void retraction() {
//        ExMotor1.setTargetPosition(ExMotor1.getCurrentPosition() - (revolution));
//        ExMotor2.setTargetPosition(ExMotor2.getCurrentPosition() - (revolution));

        ExMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ExMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ExMotor1.setPower(1);
        ExMotor2.setPower(1);

        if (touchSensor.isPressed()) {
            //unwind a little
            double timeout = getRuntime() + 0.5;
            if (timeout ==  getRuntime()) {
                ExMotor1.setPower(0.1);
                ExMotor2.setPower(0.1);

                ExMotor1.setTargetPosition(ExMotor1.getCurrentPosition() + (90));
                ExMotor2.setTargetPosition(ExMotor2.getCurrentPosition() + (90));

                ExMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ExMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if ((ExMotor1.isBusy() || ExMotor2.isBusy())) {
                    telemetry.addLine("Unwinding a little");
                } else {
                    ExMotor1.setPower(0);
                    ExMotor2.setPower(0);
                }
            } else {
                latch.setPosition(0.16);

                ExMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ExMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        isExtended = false;
    }
}
