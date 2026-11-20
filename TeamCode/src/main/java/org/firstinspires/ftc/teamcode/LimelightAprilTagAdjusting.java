package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Drive train / shooter TeleOp with the Limelight AprilTag viewer built in.
 *
 * Everything lives in this one file:
 *   - AprilTagAdjusting          : the OpMode (your original controls, unchanged)
 *   - LimelightTagViewer         : helper (nested class) that talks to the Limelight
 *   - LimelightMjpegStream       : MJPEG reader + overlay drawing (nested in the helper)
 *
 * Setup:
 *   - Limelight must be named "limelight" in Configure Robot.
 *   - If your Limelight is not at 172.29.0.1, change LIMELIGHT_HOST below.
 *   - View the annotated feed in the Driver Station menu -> Camera Stream.
 *   - If the Limelight is missing/unplugged, the robot still drives normally.
 */
@TeleOp(name = "Limelight April Tag Adjusting")
public class LimelightAprilTagAdjusting extends LinearOpMode {
    public double MovePower;

    public double ShootSpeed;
    public DcMotorEx FrontLeft;
    public DcMotorEx FrontRight;
    public DcMotorEx BackLeft;
    public DcMotorEx BackRight;
    public Servo flap;
    public DcMotor intake;
    public DcMotor transfer;
    public DcMotorEx shooter;
    public Servo ramp;

    private LimelightTagViewer viewer;   // null if the Limelight couldn't be initialized
    private int lockedTagId = -1;        // tag we're aligning to while X is held
    private boolean prevX = false;       // for detecting the X button press edge

    // ---------------- AUTO-ALIGN TUNING ----------------
    // Hold gamepad1 X to auto-center on the closest AprilTag.
    // You keep manual strafe control while it turns/drives for you.

    // Which tag to align to. Set to the ID of the tag you care about (e.g. 20).
    // -1 = "closest tag at the moment you press X", then it LOCKS to that tag
    // until you release X, so it can't jump to a different tag mid-alignment.
    private static final int TARGET_TAG_ID = -1;

    // Turning: power per degree of tx error. Raise if too slow, lower if it oscillates.
    private static final double TURN_KP = 0.02;
    private static final double MAX_TURN_POWER = 0.5;
    private static final double TX_DEADBAND_DEG = 1.0;   // "centered enough"
    private static final double TX_OFFSET_DEG = 0.0;     // if camera isn't centered on robot

    // Distance: power per meter of error.
    private static final boolean ALIGN_DISTANCE = true;
    private static final double TARGET_DISTANCE_M = 1.5;  // where you want to sit from the tag
    private static final double DRIVE_KP = 0.6;
    private static final double MAX_DRIVE_POWER = 0.4;
    private static final double DISTANCE_DEADBAND_M = 0.05;
    // ---------------------------------------------------

    @Override
    public void runOpMode() {
        MovePower = .95;
        ShootSpeed = 0.25;
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        ramp = hardwareMap.get(Servo.class, "Ramp");
        flap = hardwareMap.get(Servo.class, "Flap");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        BackLeft = hardwareMap.get(DcMotorEx.class, "BL");
        BackRight = hardwareMap.get(DcMotorEx.class, "BR");

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ramp.setPosition(.8);

        // ---- Limelight init (safe: robot still drives if this fails) ----
        try {
            viewer = new LimelightTagViewer(hardwareMap);
            telemetry.addLine("Limelight viewer ready");
        } catch (Exception e) {
            viewer = null;
            telemetry.addData("Limelight init failed", e.getMessage());
        }
        telemetry.update();

        waitForStart();

        if (viewer != null) {
            viewer.start();
        }

        try {
            while (opModeIsActive()) {

                double drive = -0.8 * gamepad1.left_stick_y;
                double strafe = -1.7 * gamepad1.left_stick_x;
                double turn = 0.8 * gamepad1.right_stick_x;

                // ---- AUTO-ALIGN: hold gamepad1 X ----
                String alignStatus = "off";

                boolean xPressed = gamepad1.x && !prevX;
                prevX = gamepad1.x;

                if (xPressed) {
                    // Lock onto a specific tag for the whole time X is held.
                    if (TARGET_TAG_ID >= 0) {
                        lockedTagId = TARGET_TAG_ID;
                    } else if (viewer != null && viewer.hasTarget()) {
                        lockedTagId = viewer.getTargetId();   // closest tag right now
                    } else {
                        lockedTagId = -1;
                    }
                }
                if (!gamepad1.x) {
                    lockedTagId = -1;
                }

                if (viewer != null) {
                    // While X is held: only the locked tag counts.
                    // Otherwise: the configured tag, or closest if TARGET_TAG_ID is -1.
                    viewer.setTagFilter(gamepad1.x && lockedTagId >= 0 ? lockedTagId : TARGET_TAG_ID);
                }

                if (gamepad1.x) {
                    if (viewer != null && viewer.hasTarget()) {
                        // If no tag was visible when X was pressed, lock to the first one seen.
                        if (lockedTagId < 0) {
                            lockedTagId = viewer.getTargetId();
                        }

                        double txError = viewer.getTx() - TX_OFFSET_DEG;

                        if (Math.abs(txError) > TX_DEADBAND_DEG) {
                            turn = clamp(TURN_KP * txError, -MAX_TURN_POWER, MAX_TURN_POWER);
                        } else {
                            turn = 0;
                        }

                        double distError = viewer.getDistanceMeters() - TARGET_DISTANCE_M;
                        if (ALIGN_DISTANCE && !Double.isNaN(distError)) {
                            if (Math.abs(distError) > DISTANCE_DEADBAND_M) {
                                drive = clamp(DRIVE_KP * distError, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
                            } else {
                                drive = 0;
                            }
                        }

                        boolean centered = Math.abs(txError) <= TX_DEADBAND_DEG;
                        alignStatus = centered ? "CENTERED" : "turning";
                    } else {
                        // No tag visible: don't move on our own, driver keeps sticks.
                        alignStatus = "no tag";
                    }
                }
                telemetry.addData("Align", alignStatus);
                float FLPower = (float) (drive + turn + strafe);
                float FRPower = (float) (drive - turn - strafe);
                float BLPower = (float) (drive + turn - strafe);
                float BRPower = (float) (drive - turn + strafe);

                FrontLeft.setPower(FLPower * MovePower);
                FrontRight.setPower(FRPower * MovePower);
                BackLeft.setPower(BLPower * MovePower);
                BackRight.setPower(BRPower * MovePower);


                if (gamepad2.right_trigger_pressed) {
                    intake.setPower(1);
                    transfer.setPower(1);
                } else if (gamepad2.left_trigger_pressed) {
                    intake.setPower(-.8);
                    transfer.setPower(-.8);
                } else if (gamepad2.y) {
                    intake.setPower(.25);
                } else {
                    intake.setPower(0);
                    transfer.setPower(0);
                }

                if (gamepad2.right_bumper) {
                    shooter.setPower(ShootSpeed);
                }

                if (gamepad2.left_bumper) {
                    shooter.setPower(0);
                }
                if (gamepad2.dpad_right) {
                    ShootSpeed = .3;
                }
                if (gamepad2.dpad_left) {
                    ShootSpeed = .35;
                }
                if (gamepad2.dpad_up) {
                    ShootSpeed = .4;
                }
                if (gamepad2.dpad_down) {
                    ShootSpeed = .15;
                }
                if (gamepad2.a) {
                    flap.setPosition(0);
                }
                if (gamepad2.b) {
                    flap.setPosition(.3);
                }
                if (gamepad1.a) {
                    ramp.setPosition(0);
                }
                if (gamepad1.b) {
                    ramp.setPosition(.8);
                }
                if (gamepad1.dpad_up) {
                    MovePower = 1.25;
                }
                if (gamepad1.dpad_left) {
                    MovePower = .95;
                }
                if (gamepad1.dpad_down) {
                    MovePower = .7;
                }

                // Adds Limelight telemetry lines (does not call telemetry.update()).
                if (viewer != null) {
                    viewer.update(telemetry);
                }

                telemetry.addData("Shoot Power", String.valueOf(ShootSpeed));
                telemetry.addData("Move Power", String.valueOf(MovePower));
                telemetry.update();
            }
        } finally {
            if (viewer != null) {
                viewer.close();
            }
        }
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // =====================================================================
    // LimelightTagViewer: non-OpMode helper
    // =====================================================================

    private static class LimelightTagViewer {

        // ---------------- USER SETTINGS ----------------
        private static final String HARDWARE_NAME = "limelight";
        private static final int PIPELINE_INDEX = 0;

        // First USB-connected Limelight commonly uses 172.29.0.1.
        private static final String LIMELIGHT_HOST = "172.29.0.1";
        private static final int STREAM_PORT = 5800;

        // Resolution of your AprilTag pipeline (used to scale corner coordinates).
        private static final int PIPELINE_WIDTH = 640;
        private static final int PIPELINE_HEIGHT = 480;

        // Limit bitmap decoding/drawing to reduce Control Hub CPU/GC pressure.
        private static final double MAX_VIEWER_FPS = 10.0;

        // FTC Limelight polling frequency.
        private static final int RESULT_POLL_HZ = 50;
        // -----------------------------------------------

        private final Limelight3A limelight;
        private final LimelightMjpegStream stream;

        // Closest tag from the most recent update() call.
        private boolean hasTarget = false;
        private int targetId = -1;
        private double targetTx = 0;
        private double targetTy = 0;
        private double targetDistanceMeters = Double.NaN;

        // -1 = consider all tags (pick closest); >= 0 = only consider this tag ID.
        private int tagFilterId = -1;

        void setTagFilter(int tagId) { tagFilterId = tagId; }

        /** True if a (matching) tag was seen in the latest update(). */
        boolean hasTarget() { return hasTarget; }
        int getTargetId() { return targetId; }
        /** Horizontal angle to tag in degrees (positive = tag is to the right). */
        double getTx() { return targetTx; }
        double getTy() { return targetTy; }
        /** Straight-line distance to tag in meters (NaN if unknown). */
        double getDistanceMeters() { return targetDistanceMeters; }

        LimelightTagViewer(HardwareMap hardwareMap) {
            limelight = hardwareMap.get(Limelight3A.class, HARDWARE_NAME);
            limelight.setPollRateHz(RESULT_POLL_HZ);
            limelight.pipelineSwitch(PIPELINE_INDEX);

            stream = new LimelightMjpegStream(
                    LIMELIGHT_HOST,
                    STREAM_PORT,
                    PIPELINE_WIDTH,
                    PIPELINE_HEIGHT,
                    MAX_VIEWER_FPS
            );
        }

        void start() {
            limelight.start();
            stream.start();

            // Makes our annotated frames available to Driver Station.
            CameraStreamServer.getInstance().setSource(stream);
            CameraStreamServer.getInstance().setJpegQuality(70);
        }

        /** Call every loop. Adds telemetry lines; does NOT call telemetry.update() or sleep(). */
        void update(Telemetry telemetry) {
            hasTarget = false;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials =
                        result.getFiducialResults();

                List<TagOverlay> overlays =
                        new ArrayList<TagOverlay>(fiducials.size());

                telemetry.addData("Tags", fiducials.size());

                for (LLResultTypes.FiducialResult fr : fiducials) {
                    TagOverlay overlay = buildOverlay(fr);
                    overlays.add(overlay);

                    // Track the closest tag as the alignment target.
                    // (Tags with unknown distance only win if nothing else was found.)
                    boolean eligible = (tagFilterId < 0 || overlay.id == tagFilterId);
                    boolean closer = eligible
                            && (!hasTarget
                            || (!Double.isNaN(overlay.distanceMeters)
                            && (Double.isNaN(targetDistanceMeters)
                            || overlay.distanceMeters < targetDistanceMeters)));
                    if (closer) {
                        hasTarget = true;
                        targetId = overlay.id;
                        targetTx = fr.getTargetXDegrees();
                        targetTy = fr.getTargetYDegrees();
                        targetDistanceMeters = overlay.distanceMeters;
                    }

                    telemetry.addData(
                            "Tag " + overlay.id,
                            "distance %.2f m / %.2f ft | tx %.1f deg | ty %.1f deg",
                            overlay.distanceMeters,
                            overlay.distanceMeters * 3.280839895,
                            fr.getTargetXDegrees(),
                            fr.getTargetYDegrees()
                    );
                }

                stream.setOverlays(Collections.unmodifiableList(overlays));

                telemetry.addData(
                        "LL Latency",
                        "%.1f ms",
                        result.getCaptureLatency()
                                + result.getTargetingLatency()
                                + result.getParseLatency()
                );
            } else {
                stream.setOverlays(Collections.<TagOverlay>emptyList());
                telemetry.addData("Tags", 0);
            }

            LLStatus status = limelight.getStatus();
            if (status != null) {
                telemetry.addData(
                        "Limelight",
                        "%s | %.1f C | %.0f%% CPU | %.0f FPS",
                        status.getName(),
                        status.getTemp(),
                        status.getCpu(),
                        status.getFps()
                );
            }

            telemetry.addData("LL connected", limelight.isConnected());
            telemetry.addData("Viewer frames", stream.getDecodedFrameCount());

            String streamError = stream.getLastError();
            if (streamError != null) {
                telemetry.addData("Stream error", streamError);
            }
        }

        void close() {
            CameraStreamServer.getInstance().setSource(null);
            stream.stop();
            limelight.stop();
        }

        /** Build one overlay from the FTC SDK Limelight fiducial result. */
        private TagOverlay buildOverlay(LLResultTypes.FiducialResult fr) {
            int id = fr.getFiducialId();

            List<List<Double>> rawCorners = fr.getTargetCorners();
            List<PointD> corners = new ArrayList<PointD>();

            if (rawCorners != null) {
                for (List<Double> point : rawCorners) {
                    if (point != null && point.size() >= 2
                            && point.get(0) != null && point.get(1) != null) {
                        corners.add(new PointD(point.get(0), point.get(1)));
                    }
                }
            }

            double distanceMeters = getCameraToTagDistanceMeters(fr);

            return new TagOverlay(id, distanceMeters, corners);
        }

        /** Straight-line 3D distance: sqrt(x^2 + y^2 + z^2) in camera space. */
        private double getCameraToTagDistanceMeters(
                LLResultTypes.FiducialResult fr) {

            try {
                Pose3D pose = fr.getTargetPoseCameraSpace();
                if (pose == null || pose.getPosition() == null) {
                    return Double.NaN;
                }

                Position p = pose.getPosition().toUnit(DistanceUnit.METER);

                return Math.sqrt(
                        p.x * p.x
                                + p.y * p.y
                                + p.z * p.z
                );
            } catch (Exception ignored) {
                return Double.NaN;
            }
        }

        // -----------------------------------------------------------------
        // Overlay data
        // -----------------------------------------------------------------

        private static class PointD {
            final double x;
            final double y;

            PointD(double x, double y) {
                this.x = x;
                this.y = y;
            }
        }

        private static class TagOverlay {
            final int id;
            final double distanceMeters;
            final List<PointD> corners;

            TagOverlay(int id, double distanceMeters, List<PointD> corners) {
                this.id = id;
                this.distanceMeters = distanceMeters;
                this.corners = new ArrayList<PointD>(corners);
            }
        }

        // -----------------------------------------------------------------
        // Limelight MJPEG -> annotated FTC CameraStreamSource
        // -----------------------------------------------------------------

        private static class LimelightMjpegStream
                implements CameraStreamSource, Runnable {

            private final String host;
            private final int port;
            private final int pipelineWidth;
            private final int pipelineHeight;
            private final long minFramePeriodNs;

            private final AtomicReference<Bitmap> latestBitmap =
                    new AtomicReference<Bitmap>();

            private volatile List<TagOverlay> overlays =
                    Collections.emptyList();

            private volatile boolean running = false;
            private volatile String lastError = null;
            private volatile long decodedFrameCount = 0;

            private Thread thread;
            private HttpURLConnection connection;

            private final Paint boxPaint = new Paint();
            private final Paint textPaint = new Paint();
            private final Paint textBackgroundPaint = new Paint();

            LimelightMjpegStream(
                    String host,
                    int port,
                    int pipelineWidth,
                    int pipelineHeight,
                    double maxFps) {

                this.host = host;
                this.port = port;
                this.pipelineWidth = pipelineWidth;
                this.pipelineHeight = pipelineHeight;

                if (maxFps <= 0) {
                    maxFps = 10.0;
                }

                this.minFramePeriodNs =
                        (long) (1_000_000_000.0 / maxFps);

                boxPaint.setColor(Color.GREEN);
                boxPaint.setStyle(Paint.Style.STROKE);
                boxPaint.setStrokeWidth(4.0f);
                boxPaint.setAntiAlias(true);

                textPaint.setColor(Color.GREEN);
                textPaint.setStyle(Paint.Style.FILL);
                textPaint.setTextSize(28.0f);
                textPaint.setAntiAlias(true);

                textBackgroundPaint.setColor(Color.argb(190, 0, 0, 0));
                textBackgroundPaint.setStyle(Paint.Style.FILL);
            }

            void setOverlays(List<TagOverlay> newOverlays) {
                if (newOverlays == null) {
                    overlays = Collections.emptyList();
                } else {
                    overlays = newOverlays;
                }
            }

            synchronized void start() {
                if (running) {
                    return;
                }

                running = true;
                thread = new Thread(this, "Limelight-MJPEG-Viewer");
                thread.setDaemon(true);
                thread.start();
            }

            synchronized void stop() {
                running = false;

                if (connection != null) {
                    connection.disconnect();
                    connection = null;
                }

                if (thread != null) {
                    thread.interrupt();
                    try {
                        thread.join(1000);
                    } catch (InterruptedException ignored) {
                        Thread.currentThread().interrupt();
                    }
                    thread = null;
                }
            }

            String getLastError() {
                return lastError;
            }

            long getDecodedFrameCount() {
                return decodedFrameCount;
            }

            @Override
            public void run() {
                while (running) {
                    try {
                        runOneConnection();
                    } catch (Exception e) {
                        lastError =
                                e.getClass().getSimpleName()
                                        + ": "
                                        + String.valueOf(e.getMessage());

                        if (connection != null) {
                            connection.disconnect();
                            connection = null;
                        }

                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException e2) {
                            Thread.currentThread().interrupt();
                            break;
                        }
                    }
                }
            }

            private void runOneConnection() throws Exception {
                URL url = new URL(
                        "http://" + host + ":" + port + "/stream.mjpg"
                );

                connection = (HttpURLConnection) url.openConnection();
                connection.setConnectTimeout(2000);
                connection.setReadTimeout(5000);
                connection.setUseCaches(false);
                connection.setRequestProperty(
                        "User-Agent",
                        "FTC-Limelight-AprilTag-Viewer"
                );
                connection.connect();

                int response = connection.getResponseCode();

                if (response < 200 || response >= 300) {
                    throw new IllegalStateException(
                            "MJPEG HTTP response " + response
                    );
                }

                InputStream input = connection.getInputStream();

                try {
                    readMjpeg(input);
                } finally {
                    try {
                        input.close();
                    } catch (Exception ignored) {
                    }

                    if (connection != null) {
                        connection.disconnect();
                        connection = null;
                    }
                }
            }

            /**
             * Locate JPEG SOI/EOI markers inside the multipart MJPEG stream.
             * This avoids needing OpenCV on the Control Hub.
             */
            private void readMjpeg(InputStream input) throws Exception {
                ByteArrayOutputStream jpeg =
                        new ByteArrayOutputStream(128 * 1024);

                boolean inJpeg = false;
                int previous = -1;
                long lastPublishedNs = 0;

                byte[] buffer = new byte[4096];

                while (running) {
                    int count = input.read(buffer);

                    if (count < 0) {
                        throw new IllegalStateException("MJPEG stream ended");
                    }

                    for (int i = 0; i < count; i++) {
                        int b = buffer[i] & 0xFF;

                        if (!inJpeg) {
                            if (previous == 0xFF && b == 0xD8) {
                                jpeg.reset();
                                jpeg.write(0xFF);
                                jpeg.write(0xD8);
                                inJpeg = true;
                            }

                            previous = b;
                            continue;
                        }

                        jpeg.write(b);

                        if (previous == 0xFF && b == 0xD9) {
                            long now = System.nanoTime();

                            if (now - lastPublishedNs >= minFramePeriodNs) {
                                byte[] data = jpeg.toByteArray();

                                Bitmap bitmap =
                                        BitmapFactory.decodeByteArray(
                                                data,
                                                0,
                                                data.length
                                        );

                                if (bitmap != null) {
                                    Bitmap annotated =
                                            annotate(bitmap, overlays);

                                    latestBitmap.set(annotated);
                                    decodedFrameCount++;
                                    lastPublishedNs = now;
                                    lastError = null;
                                }
                            }

                            jpeg.reset();
                            inJpeg = false;
                        }

                        previous = b;

                        // Corrupt/invalid stream protection.
                        if (jpeg.size() > 4 * 1024 * 1024) {
                            jpeg.reset();
                            inJpeg = false;
                        }
                    }
                }
            }

            private Bitmap annotate(
                    Bitmap source,
                    List<TagOverlay> currentOverlays) {

                Bitmap output = source.copy(
                        Bitmap.Config.ARGB_8888,
                        true
                );

                // We no longer need the decoded immutable source bitmap.
                if (output != source && !source.isRecycled()) {
                    source.recycle();
                }

                Canvas canvas = new Canvas(output);

                double scaleX =
                        pipelineWidth > 0
                                ? (double) output.getWidth() / pipelineWidth
                                : 1.0;

                double scaleY =
                        pipelineHeight > 0
                                ? (double) output.getHeight() / pipelineHeight
                                : 1.0;

                for (TagOverlay tag : currentOverlays) {
                    drawTag(canvas, tag, scaleX, scaleY);
                }

                drawStatus(canvas, currentOverlays.size());

                return output;
            }

            private void drawTag(
                    Canvas canvas,
                    TagOverlay tag,
                    double scaleX,
                    double scaleY) {

                if (tag.corners.size() >= 4) {
                    float minX = Float.MAX_VALUE;
                    float minY = Float.MAX_VALUE;

                    for (int i = 0; i < tag.corners.size(); i++) {
                        PointD a = tag.corners.get(i);
                        PointD b =
                                tag.corners.get(
                                        (i + 1) % tag.corners.size()
                                );

                        float ax = (float) (a.x * scaleX);
                        float ay = (float) (a.y * scaleY);
                        float bx = (float) (b.x * scaleX);
                        float by = (float) (b.y * scaleY);

                        canvas.drawLine(ax, ay, bx, by, boxPaint);

                        minX = Math.min(minX, ax);
                        minY = Math.min(minY, ay);
                    }

                    drawTagLabel(
                            canvas,
                            tag,
                            Math.max(4.0f, minX),
                            Math.max(32.0f, minY - 8.0f)
                    );
                } else {
                    // Corner data missing: show information in the top-left.
                    drawTagLabel(
                            canvas,
                            tag,
                            8.0f,
                            70.0f + tag.id * 34.0f
                    );
                }
            }

            private void drawTagLabel(
                    Canvas canvas,
                    TagOverlay tag,
                    float x,
                    float baselineY) {

                String distanceText;

                if (Double.isNaN(tag.distanceMeters)) {
                    distanceText = "--";
                } else {
                    distanceText =
                            String.format(
                                    java.util.Locale.US,
                                    "%.2f m / %.2f ft",
                                    tag.distanceMeters,
                                    tag.distanceMeters * 3.280839895
                            );
                }

                String label =
                        "Tag " + tag.id + " | " + distanceText;

                float width = textPaint.measureText(label);
                Paint.FontMetrics fm = textPaint.getFontMetrics();

                float top = baselineY + fm.ascent - 6.0f;
                float bottom = baselineY + fm.descent + 6.0f;

                canvas.drawRect(
                        x - 4.0f,
                        top,
                        x + width + 4.0f,
                        bottom,
                        textBackgroundPaint
                );

                canvas.drawText(label, x, baselineY, textPaint);
            }

            private void drawStatus(Canvas canvas, int tagCount) {
                String status = "AprilTags: " + tagCount;

                float width = textPaint.measureText(status);
                Paint.FontMetrics fm = textPaint.getFontMetrics();

                canvas.drawRect(
                        4.0f,
                        4.0f,
                        width + 16.0f,
                        12.0f + (fm.descent - fm.ascent),
                        textBackgroundPaint
                );

                canvas.drawText(status, 8.0f, 8.0f - fm.ascent, textPaint);
            }

            /**
             * FTC calls this when the Driver Station asks CameraStreamServer
             * for a frame.
             */
            @Override
            public void getFrameBitmap(
                    final Continuation<? extends Consumer<Bitmap>> continuation) {

                final Bitmap bitmap = latestBitmap.get();

                if (bitmap == null || bitmap.isRecycled()) {
                    return;
                }

                continuation.dispatch(
                        new ContinuationResult<Consumer<Bitmap>>() {
                            @Override
                            public void handle(Consumer<Bitmap> consumer) {
                                consumer.accept(bitmap);
                            }
                        }
                );
            }
        }
    }
}