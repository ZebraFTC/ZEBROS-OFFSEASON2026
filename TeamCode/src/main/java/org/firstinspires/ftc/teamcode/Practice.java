package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Mechanisms.DriveTrain;

@TeleOp(name = "Practice")
public class Practice extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    private DcMotor SA;
    private DcMotor MA;
    private DcMotor Bucket = null;
    private final int THREE_TWELVE_RESOLUTION = 538;
    private final int ARM_UP_TICKS = 450;
    private final int ARM_DOWN_TICKS = 0;

    @Override
    public void runOpMode(){

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        MA = hardwareMap.get(DcMotor.class, "MA" );
        SA = hardwareMap.get(DcMotor.class, "SA" );
        DriveTrain driveTrain = new DriveTrain(frontLeft, frontRight, backLeft, backRight);
        MA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MA. setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SA. setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Bucket = hardwareMap.get(DcMotor.class, "Bucket");

        int SAPosition = SA.getCurrentPosition();
        waitForStart();

        while (opModeIsActive()){
            double driveForward = -0.5 * gamepad1.left_stick_y;
            double strafeRight = -0.5 * gamepad1.left_stick_x;
            double turnCW = 0.5 * gamepad1.right_stick_x;
            driveTrain.drive(driveForward, strafeRight, turnCW);

            if (gamepad1.right_trigger > 0.1){
                Bucket.setPower(1.0);
            } else if (gamepad1.left_trigger > 0.1) {
                Bucket.setPower(-1.0);
            } else {
                Bucket.setPower(0.0);
            }

            if (gamepad1.b) {
                SA.setTargetPosition(SAPosition);
                SA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SA.setPower(0.5);
            } else if (gamepad1.a){
                SA.setTargetPosition(SAPosition-(15*THREE_TWELVE_RESOLUTION));
                SA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SA.setPower(0.3);
            }

            telemetry.addData("MA Position", MA.getCurrentPosition());
            telemetry.addData("SA Position", SA.getCurrentPosition());
            telemetry.update();

        }
    }
}


