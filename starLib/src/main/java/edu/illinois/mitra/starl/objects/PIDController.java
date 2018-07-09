package edu.illinois.mitra.starl.objects;

/**
 * Created by VerivitalLab on 1/22/2016.
 */
public class PIDController {

    // PID coefficients -- small, nonnegative coefficients that tune the controller.
    private final double Kp;
    private final double Ki;
    private final double Kd;

    //
    private final double saturationLimit;

    //
    private final double windUpLimit;

    //
    private final double[] filtArray; // the array implementing the filter
    private int filtIndex;

    private int numCommands;
    private double prevError;
    private double cumError;
    private long prevTime;


    /**
     * Constructor for a standard PID (proportional-integral-derivative) controller with
     * no modifications.
     *
     * @param Kp Proportional coefficient, implements command += Kp * error
     * @param Ki Integral coefficient, implements command += Ki * cumsum(error)
     * @param Kd Derivative coefficient, implements command += Kd * delta(error)
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, 0, 0, 1);
    }

    /**
     * Constructor for a PI (proportional-integral) controller with no modifications.
     *
     * @param Kp Proportional coefficient, implements command += Kp * error
     * @param Ki Integral coefficient, implements command += Ki * cumsum(error)
     */
    public PIDController(double Kp, double Ki) {
        this(Kp, Ki, 0);
    }

    /**
     * Constructor for a standard PID (proportional-integral-derivative) controller with
     * modifications.
     *
     * @param Kp Proportional coefficient, implements command += Kp * error
     * @param Ki Integral coefficient, implements command += Ki * cumsum(error)
     * @param Kd Derivative coefficient, implements command += Kd * delta(error)
     *
     * @param saturationLimit Represents the cap on command values this controller is allowed to produce.
     *                        If 0 or negative, command values will not be capped.
     *
     * @param windUpLimit Represents the limit on the cumulative error, capping it from growing larger in
     *                    magnitude. If 0 or negative, the cumulative error will not be capped.
     *
     * @param filterLength Represents the length of the lowpass filter (moving average) used to
     *                     compensate for high-frequency changes in the setpoint. Larger values
     *                     will smooth out the derivative term at the expense of slower response.
     *                     The default length is 1 (no smoothing), but larger lengths are recommended.
     */
    public PIDController(double Kp, double Ki, double Kd, double saturationLimit, double windUpLimit, int filterLength) {
        // initialize PID coefficients, keeping them positive
        this.Kp = Math.abs(Kp);
        this.Kd = Math.abs(Kd);
        this.Ki = Math.abs(Ki);

        // 0 is used as sentinel value to disable limiting of command values,
        // large value recommended instead
        this.saturationLimit = saturationLimit > 0.0 ? saturationLimit : 0.0;

        // 0 is used as sentinel value to disable limiting of the cumulative error value,
        // large value recommended instead
        this.windUpLimit = windUpLimit > 0.0 ? windUpLimit : 0.0;

        // length of filter array must be positive
        this.filtArray = new double[filterLength >= 1 ? filterLength : 1];
    }

    public double getCommand(double current_val, double set_point) {
        // find error (P)
        double error = set_point - current_val;

        // find change in error (junk if this is the first call to this method)
        double deltaError = error - prevError;
        prevError = error;

        // find change in time (junk if this is the first call to this method)
        long currentTime = System.nanoTime();
        long deltaTime = currentTime - prevTime;
        prevTime = currentTime;
        // convert deltaTime to double and seconds
        double deltaTimeDouble = deltaTime * 1e-9;

        // find accumulated error (I)
        cumError += error;
        // limit error if needed and if windUpLimit is not zero or negative
        if (windUpLimit > 0.0) {
            if (cumError > windUpLimit) {
                cumError = windUpLimit;
            }
            if (cumError < -windUpLimit) {
                cumError = -windUpLimit;
            }
        }

        double command = 0;

        // filter change in error, only if not first call to the method (D)
        if (numCommands > 0) {
            filtArray[filtIndex] = deltaError;
            double sum = 0;
            int currentLength = Math.min(numCommands, filtArray.length);
            for (int i = 0; i < currentLength; i++) {
                sum += filtArray[i];
            }
            double filtDeltaError = sum / currentLength;
            // increment filtIndex and reset to zero if too large
            filtIndex = (filtIndex + 1) % filtArray.length;

            // calculate D part of command value
            command = Kd * filtDeltaError / deltaTimeDouble;
        }

        // calculate and add P and I parts of command value
        command += Kp * error
                +  Ki * cumError * deltaTimeDouble;

        // limit command value if needed and if saturationLimit is not zero or negative
        if (saturationLimit > 0.0) {
            if (command > saturationLimit) {
                command = saturationLimit;
            }
            if (command < -saturationLimit) {
                command = -saturationLimit;
            }
        }
        // increment recorded number of calls to this method
        numCommands++;

        return command;
    }


    public void reset() {
        prevError = 0;
        cumError = 0;
        numCommands = 0;
        filtIndex = 0;
        prevTime = 0;
    }


    public double getKp() {
        return Kp;
    }

    public double getKi() {
        return Ki;
    }

    public double getKd() {
        return Kd;
    }

    /**
     * Represents the cap on command values this controller is allowed to produce.
     * If 0 or negative, command values will not be capped.
     */
    public double getSaturationLimit() {
        return saturationLimit;
    }

    /**
     * Represents the limit on the cumulative error, capping it from growing larger in
     * magnitude. If 0 or negative, the cumulative error will not be capped.
     */
    public double getWindUpLimit() {
        return windUpLimit;
    }

    /**
     * Represents the length of the lowpass filter (moving average) used to
     * compensate for high-frequency changes in the setpoint. Larger values
     * will smooth out the derivative term at the expense of slower response.
     * The default length is 1 (no smoothing), but larger lengths are recommended.
     */
    public double getFilterLength() {
        return filtArray.length;
    }
}
