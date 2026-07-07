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
    public DcMotor Retraction;
    public  DcMotor Extension;
    public  DcMotor intake;
    public Servo leftServo;
    public Servo rightServo;
    private boolean isExtended = false;
    int revolution = 538;  
    @Override
    public void runOpMode() {
        FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        FrontRight = hardwareMap.get(DcMotor.class, "FR");
        BackLeft = hardwareMap.get(DcMotor.class, "BL");
        BackRight = hardwareMap.get(DcMotor.class, "BR");
        Retraction = hardwareMap.get(DcMotor.class, "RE");
        Extension = hardwareMap.get(DcMotor.class, "EX");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftServo = hardwareMap.get(Servo.class, "LS");
        rightServo = hardwareMap.get(Servo.class, "RS");
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Retraction.setDirection(DcMotorSimple.Direction.REVERSE);

        Extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Retraction.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Retraction.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                Extension.setTargetPosition(Extension.getCurrentPosition() + (135));
                Retraction.setTargetPosition(Retraction.getCurrentPosition() + (135));

                Extension.setPower(1);
                Retraction.setPower(1);

                Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Retraction.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                double timeout = getRuntime() + 5.0;

                if ((Extension.isBusy()
                        || Retraction.isBusy()
                        && getRuntime() < timeout)) {

                    telemetry.addLine("Retracting");
                } else {
                    Extension.setPower(0);
                    Retraction.setPower(0);
                }

                isExtended = false;

            }
            if (gamepad1.right_bumper && isExtended == false) {
                Extension.setTargetPosition(Extension.getCurrentPosition() - (135));
                Retraction.setTargetPosition(Retraction.getCurrentPosition() - (135));

                Extension.setPower(1);
                Retraction.setPower(1);

                Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Retraction.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                double timeout = getRuntime() + 5.0;

                if ((Extension.isBusy()
                        || Retraction.isBusy()
                        && getRuntime() < timeout)) {

                    telemetry.addLine("Extending");
                } else {
                    Extension.setPower(0);
                    Retraction.setPower(0);
                }

                isExtended = true;

            }

            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            if (gamepad1.dpad_up) {
                leftServo.setPosition(0.75);
                rightServo.setPosition(0.75);
                rightServo.setDirection(Servo.Direction.REVERSE);
            }
            if (gamepad1.dpad_down) {
                leftServo.setPosition(0);
                rightServo.setPosition(0);
                rightServo.setDirection(Servo.Direction.REVERSE);
            }

        }
    }
}
