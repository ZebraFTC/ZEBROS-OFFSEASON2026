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
 * FTC Limelight AprilTag Viewer
 *
 * What it does:
 *   1. Uses the FTC SDK Limelight3A driver for AprilTag detection data.
 *   2. Reads the Limelight MJPEG stream over the Limelight USB network.
 *   3. Draws a polygon around each AprilTag.
 *   4. Draws the tag ID and straight-line camera-to-tag distance.
 *   5. Exposes the annotated bitmap through FTC CameraStreamServer so it can
 *      be viewed from the Driver Station "Camera Stream" screen.
 *
 * Hardware configuration:
 *   Add the Limelight in Configure Robot and name it exactly:
 *
 *       limelight
 *
 * Limelight setup:
 *   Configure PIPELINE_INDEX as an AprilTag/Fiducial pipeline.
 *
 * IMPORTANT:
 *   The default USB address below assumes the first Limelight on the
 *   Control Hub is reachable as 172.29.0.1.
 *   If your Limelight uses a different USB index/IP, change LIMELIGHT_HOST.
 *
 * This is intended primarily as a debugging/driver-assistance OpMode.
 * The Limelight itself performs the AprilTag detection; the Control Hub
 * only draws the overlay.
 */
@TeleOp(name = "Limelight AprilTag Viewer", group = "Vision")
public class LimelightAprilTagViewer extends LinearOpMode {

    // ---------------- USER SETTINGS ----------------

    private static final String HARDWARE_NAME = "limelight";
    private static final int PIPELINE_INDEX = 0;

    // First USB-connected Limelight on Linux/Android commonly uses 172.29.0.1.
    // If yours differs, change this value.
    private static final String LIMELIGHT_HOST = "172.29.0.1";

    // Limelight MJPEG stream.
    private static final int STREAM_PORT = 5800;

    // Set these to the resolution used by your AprilTag pipeline.
    // They are used to scale the Limelight-reported corner coordinates
    // onto the MJPEG bitmap if the displayed stream resolution differs.
    private static final int PIPELINE_WIDTH = 640;
    private static final int PIPELINE_HEIGHT = 480;

    // Limit bitmap decoding/drawing to reduce Control Hub CPU/GC pressure.
    private static final double MAX_VIEWER_FPS = 10.0;

    // FTC Limelight polling frequency.
    private static final int RESULT_POLL_HZ = 50;

    // -----------------------------------------------

    private Limelight3A limelight;
    private LimelightMjpegStream stream;

    // Immutable-ish snapshot replaced by the OpMode thread.
    private volatile List<TagOverlay> latestOverlays = Collections.emptyList();

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, HARDWARE_NAME);

        limelight.setPollRateHz(RESULT_POLL_HZ);
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.start();

        stream = new LimelightMjpegStream(
                LIMELIGHT_HOST,
                STREAM_PORT,
                PIPELINE_WIDTH,
                PIPELINE_HEIGHT,
                MAX_VIEWER_FPS
        );

        stream.start();

        // Makes our custom annotated frames available to Driver Station.
        CameraStreamServer.getInstance().setSource(stream);
        CameraStreamServer.getInstance().setJpegQuality(70);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Limelight AprilTag Viewer ready");
        telemetry.addData("Pipeline", PIPELINE_INDEX);
        telemetry.addData("Stream", "http://%s:%d/stream.mjpg",
                LIMELIGHT_HOST, STREAM_PORT);
        telemetry.addLine("Open Driver Station menu -> Camera Stream");
        telemetry.addLine("Press PLAY to continue telemetry while viewing.");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
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

                        telemetry.addData(
                                "Tag " + overlay.id,
                                "distance %.2f m / %.2f ft | tx %.1f deg | ty %.1f deg",
                                overlay.distanceMeters,
                                overlay.distanceMeters * 3.280839895,
                                fr.getTargetXDegrees(),
                                fr.getTargetYDegrees()
                        );
                    }

                    latestOverlays = Collections.unmodifiableList(overlays);
                    stream.setOverlays(latestOverlays);

                    telemetry.addData(
                            "Latency",
                            "%.1f ms",
                            result.getCaptureLatency()
                                    + result.getTargetingLatency()
                                    + result.getParseLatency()
                    );
                } else {
                    latestOverlays = Collections.emptyList();
                    stream.setOverlays(latestOverlays);
                    telemetry.addData("Tags", 0);
                    telemetry.addLine("No valid Limelight result yet");
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

                telemetry.update();
                sleep(20);
            }
        } finally {
            CameraStreamServer.getInstance().setSource(null);

            if (stream != null) {
                stream.stop();
            }

            if (limelight != null) {
                limelight.stop();
            }
        }
    }

    /**
     * Build one overlay from the FTC SDK Limelight fiducial result.
     */
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

    /**
     * Straight-line 3D distance:
     *
     *     sqrt(x^2 + y^2 + z^2)
     *
     * using the Limelight's target pose in camera space.
     */
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

    // ---------------------------------------------------------------------
    // Overlay data
    // ---------------------------------------------------------------------

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

    // ---------------------------------------------------------------------
    // Limelight MJPEG -> annotated FTC CameraStreamSource
    // ---------------------------------------------------------------------

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
            // Primary Limelight MJPEG URL.
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

                connection.disconnect();
                connection = null;
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

                    canvas.drawLine(
                            ax,
                            ay,
                            bx,
                            by,
                            boxPaint
                    );

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

            canvas.drawText(
                    label,
                    x,
                    baselineY,
                    textPaint
            );
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

            canvas.drawText(
                    status,
                    8.0f,
                    8.0f - fm.ascent,
                    textPaint
            );
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
