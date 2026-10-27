package org.firstinspires.ftc.teamcode.Mechanisms;

/**
 * First-order Linear ADRC for a velocity loop, using a discrete-time observer
 * that stays stable at any loop rate (the earlier Euler version went unstable
 * when 2*wo*dt got near or above 2, which causes jitter).
 *
 * Plant:   y' = b0*u + f      (f = total disturbance)
 * Control: u = (wc*(r - z1) - z2) / b0
 */
public class AdrcController {
    private final double b0;
    private final double wc;
    private final double wo;
    private final double filterAlpha;   // 1.0 = no filtering, smaller = smoother

    private double z1 = 0, z2 = 0;
    private double lastU = 0;
    private double yFiltered = 0;
    private boolean first = true;

    public AdrcController(double b0, double wc, double wo) {
        this(b0, wc, wo, 0.5);
    }

    public AdrcController(double b0, double wc, double wo, double filterAlpha) {
        this.b0 = b0;
        this.wc = wc;
        this.wo = wo;
        this.filterAlpha = filterAlpha;
    }

    /** @param ref desired velocity, @param measured measured velocity, @param dt seconds */
    public double update(double ref, double measured, double dt) {
        dt = Math.max(0.001, Math.min(0.05, dt));

        // Low-pass the noisy, quantized encoder velocity
        if (first) { yFiltered = measured; z1 = measured; first = false; }
        yFiltered += filterAlpha * (measured - yFiltered);

        // Discrete observer gains from a double pole at exp(-wo*dt)
        double p = Math.exp(-wo * dt);
        double l1 = 1.0 - p * p;
        double l2 = (1.0 - p) * (1.0 - p) / dt;

        // Predict with last applied power, then correct with measurement
        double z1p = z1 + dt * (z2 + b0 * lastU);
        double e = yFiltered - z1p;
        z1 = z1p + l1 * e;
        z2 += l2 * e;

        double u = (wc * (ref - z1) - z2) / b0;
        u = Math.max(-1.0, Math.min(1.0, u));

        lastU = u;
        return u;
    }

    public void reset(double measured) {
        z1 = measured;
        yFiltered = measured;
        z2 = 0;
        lastU = 0;
        first = true;
    }
}