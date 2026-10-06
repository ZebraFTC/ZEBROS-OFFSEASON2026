package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AdrcDriveTrain extends Mechanism {
    private static final double DRIVE_TO_POWER = 0.5;
    private static final double MAX_SPEED = 0.8;   // fraction of top wheel speed

    // ---- Deadband / idle ----
    private static final double STICK_DEADBAND = 0.05;     // ignore stick values below this
    private static final double IDLE_CMD_THRESHOLD = 0.02; // wheel command (fraction of max) treated as "stopped"

    // ---- ADRC tuning ----
    private static final double MOTOR_TAU = 0.10;  // approx. motor time constant (s)
    private static final double WC = 20.0;         // controller bandwidth (rad/s)
    private static final double WO = 60.0;         // observer bandwidth (rad/s), about 3x WC

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final AdrcController flCtrl, frCtrl, blCtrl, brCtrl;
    private final double maxTicksPerSec;
    private final ElapsedTime loopTimer = new ElapsedTime();

    boolean firstTime = true;
    private double xPos, yPos;

    public AdrcDriveTrain(DcMotorEx frontLeft, DcMotorEx frontRight,
                          DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        setVelocityMode();

        // Full-power wheel speed from the motor type configured in the RC app
        maxTicksPerSec = frontLeft.getMotorType().getAchieveableMaxTicksPerSecond();

        // y' = (-y + K*u)/tau  ->  input gain b0 = K/tau, with K = maxTicksPerSec
        double b0 = maxTicksPerSec / MOTOR_TAU;
        flCtrl = new AdrcController(b0, WC, WO);
        frCtrl = new AdrcController(b0, WC, WO);
        blCtrl = new AdrcController(b0, WC, WO);
        brCtrl = new AdrcController(b0, WC, WO);

        loopTimer.reset();
    }

    private void setVelocityMode() {
        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we apply power; encoders just measure
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /** Zero inside the deadband, rescaled so output still ramps smoothly from 0 to 1. */
    private static double deadband(double v, double db) {
        if (Math.abs(v) < db) return 0.0;
        return Math.signum(v) * (Math.abs(v) - db) / (1.0 - db);
    }

    /** Inputs are joystick-style values in [-1, 1]. */
    public void drive(double driveForward, double strafeRight, double turnCW) {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        driveForward = deadband(driveForward, STICK_DEADBAND) * MAX_SPEED;
        strafeRight  = deadband(strafeRight,  STICK_DEADBAND) * MAX_SPEED;
        turnCW       = deadband(turnCW,       STICK_DEADBAND) * MAX_SPEED;

        // Same mixing as your original BasicDriveTrain
        double fl = driveForward + turnCW + strafeRight;
        double fr = driveForward - turnCW - strafeRight;
        double bl = driveForward + turnCW - strafeRight;
        double br = driveForward - turnCW + strafeRight;

        // Scale all four together if any exceeds 1, so the motion direction is preserved
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        fl /= max; fr /= max; bl /= max; br /= max;

        // Idle mode: no command -> power off, observers cleared, motors brake
        if (Math.abs(fl) < IDLE_CMD_THRESHOLD && Math.abs(fr) < IDLE_CMD_THRESHOLD
                && Math.abs(bl) < IDLE_CMD_THRESHOLD && Math.abs(br) < IDLE_CMD_THRESHOLD) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            flCtrl.reset(0);
            frCtrl.reset(0);
            blCtrl.reset(0);
            brCtrl.reset(0);
            return;
        }

        frontLeft.setPower(flCtrl.update(fl * maxTicksPerSec, frontLeft.getVelocity(), dt));
        frontRight.setPower(frCtrl.update(fr * maxTicksPerSec, frontRight.getVelocity(), dt));
        backLeft.setPower(blCtrl.update(bl * maxTicksPerSec, backLeft.getVelocity(), dt));
        backRight.setPower(brCtrl.update(br * maxTicksPerSec, backRight.getVelocity(), dt));
    }

    @Override
    public void update(double time) {}

    public void setPosition(double x, double y) { xPos = x; yPos = y; }

    public boolean goToPoint(int xChange, int yChange) {
        if (firstTime) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            firstTime = false;
        }

        frontLeft.setTargetPosition(yChange + xChange);
        frontRight.setTargetPosition(yChange - xChange);   // matches the drive() mixing
        backLeft.setTargetPosition(yChange - xChange);
        backRight.setTargetPosition(yChange + xChange);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(DRIVE_TO_POWER);
        frontRight.setPower(DRIVE_TO_POWER);
        backLeft.setPower(DRIVE_TO_POWER);
        backRight.setPower(DRIVE_TO_POWER);

        if (!frontLeft.isBusy() && !frontRight.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
            firstTime = true;
            setVelocityMode();            // return to ADRC teleop mode
            flCtrl.reset(0);
            frCtrl.reset(0);
            blCtrl.reset(0);
            brCtrl.reset(0);
            return true;
        }
        return false;
    }
}