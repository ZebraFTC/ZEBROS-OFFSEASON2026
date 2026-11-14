package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Mechanisms.PIDController;

@TeleOp
public class PIDDriveTrain extends LinearOpMode {

    private double kp = 0.0003;
    private double ki = 0.0;
    private double kd = 0.001;

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    private PIDController lfPid = new PIDController(kp, ki, kd);
    private PIDController rfPid = new PIDController(kp, ki, kd);
    private PIDController lbPid = new PIDController(kp, ki, kd);
    private PIDController rbPid = new PIDController(kp, ki, kd);

    private double lastLfPos = 0, lastRfPos = 0, lastLbPos = 0, lastRbPos = 0;
    private com.qualcomm.robotcore.util.ElapsedTime velocityTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    private final double MAX_TICKS_PER_SECOND = 250.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Motors as DcMotorEx
        leftFront  = hardwareMap.get(DcMotorEx.class, "FL");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");
        leftBack   = hardwareMap.get(DcMotorEx.class, "BL");
        rightBack  = hardwareMap.get(DcMotorEx.class, "BR");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        velocityTimer.reset();
        lfPid.reset(); rfPid.reset(); lbPid.reset(); rbPid.reset();

        lastLfPos = leftFront.getCurrentPosition();
        lastRfPos = rightFront.getCurrentPosition();
        lastLbPos = leftBack.getCurrentPosition();
        lastRbPos = rightBack.getCurrentPosition();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double lfPowerTarget = y + x + rx;
            double rfPowerTarget = y - x - rx;
            double lbPowerTarget = y - x + rx;
            double rbPowerTarget = y + x - rx;

            double maxPower = Math.max(
                    Math.max(Math.abs(lfPowerTarget), Math.abs(rfPowerTarget)),
                    Math.max(Math.abs(lbPowerTarget), Math.abs(rbPowerTarget))
            );

            if (maxPower > 1.0) {
                lfPowerTarget /= maxPower;
                rfPowerTarget /= maxPower;
                lbPowerTarget /= maxPower;
                rbPowerTarget /= maxPower;
            }

            double lfTargetVel = lfPowerTarget * MAX_TICKS_PER_SECOND;
            double rfTargetVel = rfPowerTarget * MAX_TICKS_PER_SECOND;
            double lbTargetVel = lbPowerTarget * MAX_TICKS_PER_SECOND;
            double rbTargetVel = rbPowerTarget * MAX_TICKS_PER_SECOND;

            double dt = velocityTimer.seconds();
            if (dt <= 0) dt = 0.001;
            velocityTimer.reset();

            double currentLfPos = leftFront.getCurrentPosition();
            double currentRfPos = rightFront.getCurrentPosition();
            double currentLbPos = leftBack.getCurrentPosition();
            double currentRbPos = rightBack.getCurrentPosition();

            double lfCurrentVel = (currentLfPos - lastLfPos) / dt;
            double rfCurrentVel = (currentRfPos - lastRfPos) / dt;
            double lbCurrentVel = (currentLbPos - lastLbPos) / dt;
            double rbCurrentVel = (currentRbPos - lastRbPos) / dt;

            lastLfPos = currentLfPos;
            lastRfPos = currentRfPos;
            lastLbPos = currentLbPos;
            lastRbPos = currentRbPos;

            double lfOutputPower = lfPid.calculate(lfTargetVel, lfCurrentVel);
            double rfOutputPower = rfPid.calculate(rfTargetVel, rfCurrentVel);
            double lbOutputPower = lbPid.calculate(lbTargetVel, lbCurrentVel);
            double rbOutputPower = rbPid.calculate(rbTargetVel, rbCurrentVel);

            leftFront.setPower(lfOutputPower);
            rightFront.setPower(rfOutputPower);
            leftBack.setPower(lbOutputPower);
            rightBack.setPower(rbOutputPower);

        }
    }
}
