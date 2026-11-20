package org.firstinspires.ftc.teamcode.Mechanisms;

public class ADRC_driver {
    private double b0;
    private double beta1, beta2;
    private double kp;

    private double z1 = 0.0; // Estimated velocity (ticks/sec)
    private double z2 = 0.0; // Estimated total disturbance
    private double lastOutput = 0.0;

    public ADRC_driver(double b0, double omegaO, double omegaC) {
        this.b0 = b0;
        this.beta1 = 2.0 * omegaO;
        this.beta2 = omegaO * omegaO;
        this.kp = omegaC;
    }

    public void reset(double currentVelocity) {
        z1 = currentVelocity;
        z2 = 0.0;
        lastOutput = 0.0;
    }

    public double calculate(double targetVelocity, double currentVelocity, double dt) {
        if (dt <= 0 || dt > 0.5) return lastOutput; // Guard against bad dt spikes

        // 1. Update 2nd-order Extended State Observer for velocity
        double error = currentVelocity - z1;
        z1 += dt * (z2 + beta1 * error + b0 * lastOutput);
        z2 += dt * (beta2 * error);

        // 2. Proportional control law + disturbance cancellation
        double u0 = kp * (targetVelocity - z1);
        double u = (u0 - z2) / b0;

        // 3. Clamp power between -1.0 and 1.0 (or 0.0 and 1.0 for one-way shooter)
        u = Math.max(0.0, Math.min(1.0, u));
        lastOutput = u;
        return u;
    }
}
