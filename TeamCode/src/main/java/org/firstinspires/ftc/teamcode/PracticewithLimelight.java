package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Mechanisms.DriveTrain;

@TeleOp(name = "PracticewithLimelight")
public class PracticewithLimelight extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    private DcMotor SA;
    private DcMotor MA;
    private DcMotor Bucket;
    private final int THREE_TWELVE_RESOLUTION = 538;
    private final int ARM_UP_TICKS = 450;
    private final int ARM_DOWN_TICKS = 0;
    LimelightAprilTagViewer Camera = new LimelightAprilTagViewer();

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
        MA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Bucket = hardwareMap.get(DcMotor.class, "Bucket");

        waitForStart();

        while (opModeIsActive()){
            double driveForward = -0.8 * gamepad1.left_stick_y;
            double strafeRight = -0.8 * gamepad1.left_stick_x;
            double turnCW = 0.8 * gamepad1.right_stick_x;
            driveTrain.drive(driveForward, strafeRight, turnCW);

            if (gamepad1.right_trigger > 0.1){
                Bucket.setPower(1.0);
            } else if (gamepad1.left_trigger > 0.1) {
                Bucket.setPower(-1.0);
            } else {
                Bucket.setPower(0.0);
            }

            if (gamepad1.b) {
                //ideal position
                MA.setTargetPosition(0);
                MA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MA.setPower(0.8);
                SA.setTargetPosition(0);
                SA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SA.setPower(0.8);
            } else if (gamepad1.a) {
                //intake position
                MA.setTargetPosition(829);
                MA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MA.setPower(0.8);
                SA.setTargetPosition(1586);
                SA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SA.setPower(0.8);
            } else if (gamepad1.x) {
                //regular scoring
                MA.setTargetPosition(5172);
                MA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MA.setPower(0.8);
                SA.setTargetPosition(-1053);
                SA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SA.setPower(0.8);
            } else if (gamepad1.y) {
                //top bucket scoring red
                MA.setTargetPosition(3408);
                MA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MA.setPower(0.8);
                SA.setTargetPosition(120);
                SA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SA.setPower(0.8);
            } else if (gamepad1.left_bumper) {
                //straight scoring
                MA.setTargetPosition(1768);
                MA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MA.setPower(0.8);
                SA.setTargetPosition(1055);
                SA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SA.setPower(0.8);
            } else if (gamepad1.right_bumper) {
                //top bucket scoring blue
                MA.setTargetPosition(3522);
                MA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MA.setPower(0.8);
                SA.setTargetPosition(583);
                SA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SA.setPower(0.8);
            }

            telemetry.addData("MA Position", MA.getCurrentPosition());
            telemetry.addData("SA Position", SA.getCurrentPosition());
            telemetry.update();

        }
    }
}