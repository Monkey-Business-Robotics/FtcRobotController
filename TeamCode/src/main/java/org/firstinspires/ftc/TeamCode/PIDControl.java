package org.firstinspires.ftc.TeamCode;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDControl {

    private final double tolerance;

    private final PIDCoefficients PID;
    private double prevError;
    private double integralSum;

    public PIDControl(PIDCoefficients PID) {
        this(PID, 1);
    }

    public PIDControl(PIDCoefficients PID, double tolerance) {
        this.PID = PID;
        this.tolerance = tolerance;
    }

    public void reset() {
        this.prevError = 0;
        this.integralSum = 0;
    }

    public double update(double targetPosition, double currentPosition) {
        // Calculate error
        double error = targetPosition - Math.abs(currentPosition);

        // Proportional term
        double proportional = PID.p * error;

        // Integral term
        integralSum += error;
        double integral = PID.i * integralSum;

        // Derivative term
        double derivative = error - prevError;
        double derivativeTerm = PID.d * derivative;

        // Combine PID terms
        double output = proportional + integral + derivativeTerm;

        double distanceToTarget = Math.abs(error);
        if (distanceToTarget < 2 * this.tolerance) {
            output *=distanceToTarget / 10 * this.tolerance;
        }

        // Clamp output to valid motor power range [-1, 1]
        output = Math.max(-1, Math.min(1, output));

        // Update previous error for the next cycle
        prevError = error;

        // Stop the motor if within tolerance
        if (Math.abs(error) <= this.tolerance) {
            output = (0);
        }

        return output;
    }
}
