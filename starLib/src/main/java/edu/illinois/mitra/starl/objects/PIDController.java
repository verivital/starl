package edu.illinois.mitra.starl.objects;

import java.util.Arrays;

/**
 * A PID (proportional-integral-derivative) controller implementation, with
 * additional modifications of saturation limit, wind-up limit, and lowpass
 * filtering of the derivative signal.
 *
 * Given a setpoint value and a current value, the PID controller will return command values
 * that smoothly bring the current value to the setpoint value, depending on the three constants
 * Kp, Ki, and Kd. See https://en.wikipedia.org/wiki/PID_controller#Loop_tuning for information
 * on how to find optimal values.
 *
 * Created by VerivitalLab on 1/22/2016, updated 7/10/18.
 */
public class PIDController {

    // PID coefficients -- small, nonnegative coefficients that tune the controller.
    private final double Kp;
    private final double Ki;
    private final double Kd;

    // see getSaturationLimit(), getWindUpLimit(), getFilterLength()
    private final double saturationLimit;
    private final double windUpLimit;

    private final double[] filtArray; // the array implementing the filter ring buffer
    private double filtRunningSum; // the running sum of the elements in filtArray
    private int filtIndex; // the next index to write to in filtArray

    private boolean notFirstCall; // true implies prevError and prevTime are valid
    private double prevError; // the error recorded on the last call to getCommand()
    private double cumError; // the accumulated error so far
    private long prevTime; // the time recorded on the last call to getCommand()


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
     * Constructor for a standard PI (proportional-integral) controller with no
     * modifications. The derivative coefficient is often not necessary.
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

    /**
     * Execute the PID algorithm. Call repeatedly and apply the returned value to the actuator,
     * in order to physically move the current value closer to the set point.
     *
     * @param currentVal the current value of some system property
     * @param setPoint the desired value for the system property
     * @return a command value used to bring the current value closer to the set point
     */
    public double getCommand(double currentVal, double setPoint) {
        return getCommand(setPoint - currentVal);
    }

    /**
     * Execute the PID algorithm. Call repeatedly and apply the returned value to the actuator,
     * in order to physically move the current value closer to the set point.
     *
     * @param error the difference between the desired value and the current value
     *              of some system property
     * @return a command value used to reduce the error
     */
    public double getCommand(double error) {
        // find change in error (junk if this is the first call to this method)
        double deltaError = error - prevError;
        prevError = error;

        // find change in time (0 if this is the first call to this method)
        double deltaTime = getDeltaTime();

        // calculate the command value, adding the P, I, and D components
        double command = getPComponent(error)
                       + getIComponent(error, deltaTime)
                       + getDComponent(deltaError, deltaTime);

        // limit command value if needed
        command = cap(command, saturationLimit);
        // lastly, record that this method has been called => prevError and prevTime are valid
        notFirstCall = true;

        return command;
    }

    /**
     * Erase the controller's history, making it function as if it had just been instantiated.
     */
    public void reset() {
        Arrays.fill(filtArray, 0.0);
        filtRunningSum = 0;
        filtIndex = 0;
        notFirstCall = false;
        prevError = 0;
        cumError = 0;
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
     * The default length is 1 (no smoothing), but larger lengths (~8) are recommended.
     */
    public double getFilterLength() {
        return filtArray.length;
    }


    private double getDeltaTime() {
        // find change in time (0 if this is the first call to this method)
        long currentTime = System.nanoTime();
        long deltaTimeLong = currentTime - prevTime;
        prevTime = currentTime;
        // convert deltaTime to double and seconds
        return (notFirstCall ? deltaTimeLong * 1e-9 : 0);
    }

    private double getPComponent(double error) {
        // calculate P component of command value
        return Kp * error;
    }

    private double getIComponent(double error, double deltaTime) {
        // find accumulated error
        cumError += error * deltaTime;
        // limit error if needed
        cumError = cap(cumError, windUpLimit);
        // calculate I component of command value
        return Ki * cumError;
    }

    private double getDComponent(double deltaError, double deltaTime) {
        if (notFirstCall) {
            // filter change in error using a running sum
            filtArray[filtIndex] = deltaError;
            // add newest entry
            filtRunningSum += filtArray[filtIndex];
            // increment filtIndex and reset to zero if too large, index of oldest entry or 0
            filtIndex = (filtIndex + 1) % filtArray.length;
            // subtract oldest entry or 0
            filtRunningSum -= filtArray[filtIndex];

            // calculate D component of command value
            return Kd * filtRunningSum / (filtArray.length * deltaTime);
        }
        return 0;
    }

    private static double cap(double value, double absLimit) {
        // limits the absolute value of 'value' to positive or negative absLimit, if absLimit is positive
        if (absLimit > 0) {
            if (value > absLimit) {
                return absLimit;
            }
            if (value < -absLimit) {
                return -absLimit;
            }
        }
        return value;
    }
}
