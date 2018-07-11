package edu.illinois.mitra.starl.objects;

import java.util.Arrays;

/**
 * A PID (proportional-integral-derivative) controller implementation, with
 * additional modifications of saturation limit, wind-up limit, output slope limit,
 * stored setpoint, and lowpass filtering of the derivative signal.
 *
 * Given a setpoint value and a current value, the PID controller will return command values
 * that smoothly bring the current value to the setpoint value, depending on the three constants
 * Kp, Ki, and Kd. See https://en.wikipedia.org/wiki/PID_controller#Loop_tuning for information
 * on how to find optimal values.
 *
 * Created by VerivitalLab on 1/22/2016, updated 7/10/18.
 * Some inspiration from https://github.com/tekdemo/MiniPID-Java.
 */
public class PIDController {

    // PID coefficients -- small, nonnegative coefficients that tune the controller.
    private double Kp;
    private double Ki;
    private double Kd;

    // see configuration methods
    private double saturationLimit; // the maximum absolute value of the output signal
    private double windUpLimit; // the maximum allowed accumulated error
    private double outputSlopeLimit; // the maximum allowed slope of the output signal, per second
    private double setpoint; // the stored set point
    private boolean reversed; // negate the output values

    private double[] filtArray; // the array implementing the filter ring buffer
    private double filtRunningSum; // the running sum of the elements in filtArray
    private int filtIndex; // the next index to write to in filtArray

    // temporary data
    private boolean notFirstCall; // true implies prevError and prevTime are valid
    private double prevError; // the error recorded on the last call to getOutput()
    private double cumError; // the accumulated error so far
    private long prevTime; // the time recorded on the last call to getOutput()
    private double prevOutput; // the last recorded output (not reversed)


    /**
     * Constructor for a standard PID (proportional-integral-derivative) controller with
     * no modifications.
     *
     * @param Kp Proportional coefficient, implements command += Kp * error
     * @param Ki Integral coefficient, implements command += Ki * cumsum(error)
     * @param Kd Derivative coefficient, implements command += Kd * delta(error)
     */
    public PIDController(double Kp, double Ki, double Kd) {
        setKp(Kp);
        setKi(Ki);
        setKd(Kd);
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
     * @param params PIDParams object containing all relevant parameters for a PID controller
     */
    public PIDController(PIDParams params) {
        this(params.Kp, params.Ki, params.Kd);
        setSaturationLimit(params.saturationLimit);
        setWindUpLimit(params.windUpLimit);
        setFilterLength(params.filterLength);
        setSetpoint(params.setpoint);
        setOutputSlopeLimit(params.outputSlopeLimit);
        setReversed(params.reversed);
    }

    /**
     * Execute the PID algorithm, using the last recorded setpoint. Call repeatedly and
     * apply the returned value to the actuator, in order to physically move the current
     * value closer to the set point.
     *
     * @param currentVal the current value of some system property
     * @return a command value used to bring the current value closer to the set point
     */
    public double getOutput(double currentVal) {
        return getOutput(currentVal, getSetpoint());
    }

    /**
     * Execute the PID algorithm. Call repeatedly and apply the returned value to the actuator,
     * in order to physically move the current value closer to the set point.
     *
     * @param currentVal the current value of some system property
     * @param setpoint the desired value for the system property
     * @return a command value used to bring the current value closer to the set point
     */
    public double getOutput(double currentVal, double setpoint) {
        setSetpoint(setpoint);
        // find error
        double error = setpoint - currentVal;

        // find change in error (junk if this is the first call to this method)
        double deltaError = error - prevError;
        prevError = error;

        // find change in time (0 if this is the first call to this method)
        double deltaTime = getDeltaTime();

        // calculate the output value, adding the P, I, and D components
        double output = getPComponent(error)
                       + getIComponent(error, deltaTime)
                       + getDComponent(deltaError, deltaTime);

        // limit output value if needed
        output = cap(output, saturationLimit); // limits to absolute range
        if (notFirstCall) {
            output = cap(output, outputSlopeLimit * deltaTime, prevOutput); // limits slope of output
        }
        prevOutput = output;

        // reverse output value if needed
        if (reversed) {
            output = -output;
        }

        // lastly, record that this method has been called => prev* variables are valid
        notFirstCall = true;
        return output;
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
        setpoint = 0;
    }

    /**
     * Access the P coefficient. Setting takes the absolute value.
     */
    public double getKp() {
        return Kp;
    }
    public void setKp(double Kp) {
        this.Kp = Math.abs(Kp);
    }

    /**
     * Access the I coefficient. Setting takes the absolute value. The cumulative
     * error is scaled so that the output does not spike.
     */
    public double getKi() {
        return Ki;
    }
    public void setKi(double Ki) {
        Ki = Math.abs(Ki);
        // scale cumError so that the output does not change instantaneously
        if (Ki != 0) {
            cumError *= this.Ki / Ki;
        }
        cumError = cap(cumError, windUpLimit);
        this.Ki = Ki;
    }

    /**
     * Access the D coefficient. Setting takes the absolute value.
     */
    public double getKd() {
        return Kd;
    }
    public void setKd(double Kd) {
        this.Kd = Math.abs(Kd);
    }

    /**
     * Represents the cap on command values this controller is allowed to produce.
     * If 0 or negative, command values will not be capped.
     */
    public double getSaturationLimit() {
        return saturationLimit;
    }
    public void setSaturationLimit(double saturationLimit) {
        // 0 is used as sentinel value to disable limiting of command values,
        // large value recommended instead
        this.saturationLimit = saturationLimit > 0.0 ? saturationLimit : 0.0;
    }

    /**
     * Represents the limit on the cumulative error, capping it from growing larger in
     * magnitude. If 0 or negative, the cumulative error will not be capped.
     */
    public double getWindUpLimit() {
        return windUpLimit;
    }
    public void setWindUpLimit(double windUpLimit) {
        // 0 is used as sentinel value to disable limiting of the cumulative error value,
        // large value recommended instead
        this.windUpLimit = windUpLimit > 0.0 ? windUpLimit : 0.0;
        cumError = cap(cumError, this.windUpLimit);
    }

    /**
     * Represents the length of the lowpass filter (moving average) used to
     * compensate for high-frequency changes in the setpoint. Larger values
     * will smooth out the derivative term at the expense of slower response.
     * The default length is 1 (no smoothing), but larger lengths (~8) are recommended.
     *
     * Note: setting the filter length resets the D component of the controller and
     * should only be used during initialization/configuration.
     */
    public double getFilterLength() {
        return filtArray.length;
    }
    public void setFilterLength(int filterLength) {
        // length of filter array must be positive
        filtArray = new double[filterLength >= 1 ? filterLength : 1];
        if (filtRunningSum != 0) {
            // fill array with equivalent average value to avoid a sharp jump
            Arrays.fill(filtArray, filtRunningSum / filtArray.length);
        }
        filtIndex = 0;
    }

    /**
     * Represents the limit on how quickly the output is allowed to change, in units
     * per second.
     */
    public double getOutputSlopeLimit() {
        return outputSlopeLimit;
    }
    public void setOutputSlopeLimit(double outputSlopeLimit) {
        this.outputSlopeLimit = Math.abs(outputSlopeLimit);
    }

    /**
     * Returns the current setpoint, which was set with either setSetpoint() or
     * getOutput(setpoint).
     */
    public double getSetpoint() {
        return setpoint;
    }
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Returns whether the output is negated or not
     */
    public boolean isReversed() {
        return reversed;
    }
    public void setReversed(boolean reversed) {
        this.reversed = reversed;
    }


    /*
     * Private methods
     */
    private double getDeltaTime() {
        long currentTime = System.nanoTime();
        // find change in time (0 if this is the first call to this method)
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
            // increment filtIndex and reset to zero if too large, index of oldest entry
            filtIndex = (filtIndex + 1) % filtArray.length;
            // subtract oldest entry
            filtRunningSum -= filtArray[filtIndex];

            // calculate D component of command value
            return Kd * filtRunningSum / (filtArray.length * deltaTime);
        }
        return 0;
    }

    private static double cap(double value, double absLimit) {
        // limits the absolute value of 'value' to positive or negative absLimit, if absLimit is positive
        return cap(value, absLimit, 0.0);
    }

    private static double cap(double value, double absLimit, double center) {
        // limits the absolute value of 'value' to center plus or minus absLimit, if absLimit is positive
        if (absLimit > 0) {
            if (value > center + absLimit) {
                return center + absLimit;
            }
            if (value < center - absLimit) {
                return center - absLimit;
            }
        }
        return value;
    }
}
