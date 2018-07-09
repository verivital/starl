package edu.illinois.mitra.starl.motion;

/**
 * Created by VerivitalLab on 1/22/2016.
 */
public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;
    private double saturationLimit;
    private double windUpLimit;
    private int filterLength;
    // filtArray is used as a window filter
    private double[] filtArray;
    private int filtIndex;
    private int numCommands;
    private double prevError;
    private double error;
    private double cumError;
    private double deltaError;
    private double filtDeltaError;
    private long prevTime;



    public PIDController(double Kp, double Ki, double Kd, double saturationLimit, double windUpLimit, int filterLength) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.saturationLimit = saturationLimit;
        this.windUpLimit = windUpLimit;
        this.filterLength = filterLength;
        this.filtArray = new double[filterLength];
        error = 0;
        prevError = 0;
        cumError = 0;
        deltaError = 0;
        numCommands = 0;
        filtIndex = 0;
        filtDeltaError = 0;
        prevTime = 0;
    }

    public double getCommand(double current_val, double set_point) {
        int i;
        // find error
        error = set_point - current_val;
        // if first time getCommand called, set prev error and time, return 0 command
        if (numCommands == 0) {
            prevError = error;
            prevTime = System.nanoTime();
            numCommands++;
            return 0;
        }

        // find change in error
        deltaError = error - prevError;
        prevError = error;

        // filter change in error (window type filter)
        filtArray[filtIndex] = deltaError;
        double sum = 0;
        int currentLength = Math.min(numCommands, filterLength);
        for(i = 0; i < currentLength; i++) {
            sum += filtArray[i];
        }
        filtDeltaError = sum/currentLength;
        // increment filtIndex and reset to zero if too large
        filtIndex++;
        if(filtIndex >= filterLength) {
            filtIndex = 0;
        }

        // find change in time
        long currentTime = System.nanoTime();
        long deltaTime = currentTime - prevTime;
        prevTime = currentTime;
        // convert deltaTime to double and seconds
        double deltaTimeDouble = deltaTime * 1e-9;

        // find accumulated error
        cumError += error;
        // limit error if needed
        if (cumError > windUpLimit) {
            cumError = windUpLimit;
        }
        if (cumError < -windUpLimit) {
            cumError = -windUpLimit;
        }
        // calculated command
        double command = (Kp * error) + (Ki * cumError * deltaTimeDouble) + (Kd * filtDeltaError * (1/deltaTimeDouble));
        // limit command value if needed
        if(command > saturationLimit) {
            command = saturationLimit;
        }
        if(command < -saturationLimit) {
            command = -saturationLimit;
        }
        numCommands++;
        return command;
    }


    public void reset() {
        error = 0;
        prevError = 0;
        cumError = 0;
        deltaError = 0;
        numCommands = 0;
        filtIndex = 0;
        filtDeltaError = 0;
        prevTime = 0;
    }




}
