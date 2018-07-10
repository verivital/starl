package edu.illinois.mitra.starl.objects;

public final class PIDParams {
    // PID coefficients -- small, nonnegative coefficients that tune the controller.
    public double Kp;
    public double Ki;
    public double Kd;
    public double saturationLimit;
    public double windUpLimit;
    public int filterLength;

    public PIDParams() {
        this(1, 0, 0);
    }

    public PIDParams(double Kp, double Ki) {
        this(Kp, Ki, 0);
    }

    public PIDParams(double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, 0, 0, 1);
    }

    public PIDParams(double Kp, double Ki, double Kd, double saturationLimit, double windUpLimit, int filterLength) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.saturationLimit = saturationLimit;
        this.windUpLimit = windUpLimit;
        this.filterLength = filterLength;
    }
}
