package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BasicDriveTrain extends LinearOpMode {
    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor BackLeft;
    public DcMotor BackRight;

    @Override
    public void runOpMode() {
        FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        FrontRight = hardwareMap.get(DcMotor.class, "FR");
        BackLeft = hardwareMap.get(DcMotor.class, "BL");
        BackRight = hardwareMap.get(DcMotor.class, "BR");
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {

            double drive = -0.8 * gamepad1.left_stick_y;
            double strafe = -0.8 * gamepad1.left_stick_x;
            double turn = 0.8 * gamepad1.right_stick_x;
            FrontLeft.setPower(drive + turn + strafe);
            FrontRight.setPower(drive - turn + strafe);
            BackLeft.setPower(drive + turn - strafe);
            BackRight.setPower(drive - turn - strafe);


        }
    }
}