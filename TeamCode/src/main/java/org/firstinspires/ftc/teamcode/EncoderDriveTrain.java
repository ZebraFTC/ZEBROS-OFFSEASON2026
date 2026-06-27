package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class EncoderDriveTrain extends LinearOpMode {
    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor BackLeft;
    public DcMotor BackRight;
    public DcMotor Retraction;
    public  DcMotor Extension;
    int revolution = 538;
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
            FrontRight.setPower(drive-turn-strafe);
            BackLeft.setPower(drive+turn-strafe);
            BackRight.setPower(drive-turn+strafe);

            if (gamepad1.left_trigger > 0.2) {
                Extension.setTargetPosition(Extension.getCurrentPosition() + (135));
                Retraction.setTargetPosition(Retraction.getCurrentPosition() + (135));

                Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Retraction.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Extension.setPower(0.75);
                Retraction.setPower(0.75);

                double timeout = getRuntime() + 5.0;

                if ((Extension.isBusy()
                        || Retraction.isBusy()
                        && getRuntime() < timeout)) {

                    telemetry.addLine("Retracting");
                } else {
                    Extension.setPower(0);
                    Retraction.setPower(0);
                }


            }
            if (gamepad1.right_trigger > 0.2) {
                Extension.setTargetPosition(Extension.getCurrentPosition() - (135));
                Retraction.setTargetPosition(Retraction.getCurrentPosition() - (135));

                Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Retraction.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Extension.setPower(1);
                Retraction.setPower(1);

                double timeout = getRuntime() + 5.0;

                if ((Extension.isBusy()
                        || Retraction.isBusy()
                        && getRuntime() < timeout)) {

                    telemetry.addLine("Extending");
                } else {
                    Extension.setPower(0);
                    Retraction.setPower(0);
                }
            }

        }
    }
}
