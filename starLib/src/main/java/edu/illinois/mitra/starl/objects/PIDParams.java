package edu.illinois.mitra.starl.objects;

/**
 * A small, plain-old-data helper class to initialize PIDController instances.
 *
 * Kp, Ki, and Kd should be positive.
 * Zero values for the other numerical parameters will result in default behavior,
 * with no features added. Setting true values will enable features like saturation
 * limit, wind-up limit, derivative signal filtering, and output slope limiting.
 */
public final class PIDParams {
    // PID coefficients -- small, nonnegative coefficients that tune the controller.
    public double Kp;
    public double Ki;
    public double Kd;
    public double saturationLimit;
    public double windUpLimit;
    public int filterLength;
    public double setpoint;
    public double outputSlopeLimit;
    public boolean reversed;
}
