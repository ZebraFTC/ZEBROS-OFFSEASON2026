package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kp;
    private double ki;
    private double kd;

    private double integralSum = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        timer.reset();
    }

    public double calculate(double reference, double state) {
        double error = reference - state;

        double deltaTime = timer.seconds();

        if (deltaTime <= 0) {
            deltaTime = 0.001;
        }

        // Rate of change of the error (Derivative)
        double derivative = (error - lastError) / deltaTime;

        integralSum += error * deltaTime;

        double output = (kp * error) + (ki * integralSum) + (kd * derivative);

        lastError = error;
        timer.reset();

        return output;
    }

    public void setGains(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}
