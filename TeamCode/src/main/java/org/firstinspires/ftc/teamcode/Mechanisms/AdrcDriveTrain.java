
package org.firstinspires.ftc.teamcode.Mechanisms;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AdrcDriveTrain extends Mechanism {
    private static final double DRIVE_TO_POWER = 0.5;

    // INCREASED MOTOR SPEED: Raised from 0.8 to 1.0 to unleash full motor capacity
    private static final double MAX_SPEED = 1.0;

    // ---- Deadband / idle ----
    private static final double STICK_DEADBAND = 0.05;
    private static final double IDLE_CMD_THRESHOLD = 0.02;

    // ---- REDUCED CHATTER TUNING ----
    // Lowered bandwidths smooth out high-frequency encoder noise and stop structural chatter.
    // Kept the golden 3x to 5x separation rule (WO ≈ 3-4 * WC).
    public static double WC = 7.0;         // Controller bandwidth (rad/s) - lowered from 10.0
    public static double WO = 25.0;        // Observer bandwidth (rad/s) - adjusted from 20.0

    // ---- Saturation Protection ----
    // Capping max power commands preventing calculation windup that triggers aggressive oscillation.
    private static final double MAX_OUTPUT_POWER = 1.0;

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

        maxTicksPerSec = frontLeft.getMotorType().getAchieveableMaxTicksPerSecond();

        // System gains (Adjust MOTOR_TAU slightly up if the system feels overly snappy or hyperactive)
        double b0 = maxTicksPerSec / 0.12;
        flCtrl = new AdrcController(b0, WC, WO);
        frCtrl = new AdrcController(b0, WC, WO);
        blCtrl = new AdrcController(b0, WC, WO);
        brCtrl = new AdrcController(b0, WC, WO);

        loopTimer.reset();
    }

    private void setVelocityMode() {
        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private static double deadband(double v, double db) {
        if (Math.abs(v) < db) return 0.0;
        return Math.signum(v) * (Math.abs(v) - db) / (1.0 - db);
    }

    public void drive(double driveForward, double strafeRight, double turnCW) {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // TIMING PROTECTION: Guard against zero/near-zero loops which break derivative tracking and cause massive chatter
        if (dt < 0.0005) {
            dt = 0.005; // Fallback to a healthy baseline 5ms step size
        }

        driveForward = deadband(driveForward, STICK_DEADBAND) * MAX_SPEED;
        strafeRight  = deadband(strafeRight,  STICK_DEADBAND) * MAX_SPEED;
        turnCW       = deadband(turnCW,       STICK_DEADBAND) * MAX_SPEED;

        double fl = driveForward + turnCW + strafeRight;
        double fr = driveForward - turnCW - strafeRight;
        double bl = driveForward + turnCW - strafeRight;
        double br = driveForward - turnCW + strafeRight;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        fl /= max; fr /= max; bl /= max; br /= max;

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

        // CONTROL LOOPS WITH CLAMPED SATURATION PROTECTION
        double pFL = com.qualcomm.robotcore.util.Range.clip(flCtrl.update(fl * maxTicksPerSec, frontLeft.getVelocity(), dt), -MAX_OUTPUT_POWER, MAX_OUTPUT_POWER);
        double pFR = com.qualcomm.robotcore.util.Range.clip(frCtrl.update(fr * maxTicksPerSec, frontRight.getVelocity(), dt), -MAX_OUTPUT_POWER, MAX_OUTPUT_POWER);
        double pBL = com.qualcomm.robotcore.util.Range.clip(blCtrl.update(bl * maxTicksPerSec, backLeft.getVelocity(), dt), -MAX_OUTPUT_POWER, MAX_OUTPUT_POWER);
        double pBR = com.qualcomm.robotcore.util.Range.clip(brCtrl.update(br * maxTicksPerSec, backRight.getVelocity(), dt), -MAX_OUTPUT_POWER, MAX_OUTPUT_POWER);

        frontLeft.setPower(pFL);
        frontRight.setPower(pFR);
        backLeft.setPower(pBL);
        backRight.setPower(pBR);
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
        frontRight.setTargetPosition(yChange - xChange);
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
            setVelocityMode();
            flCtrl.reset(0);
            frCtrl.reset(0);
            blCtrl.reset(0);
            brCtrl.reset(0);
            return true;
        }
        return false;
    }
}
