package edu.illinois.mitra.starl.modelinterfaces;

/**
 * The interface for communicating with zero-point ground robots.
 *
 * @see ModelInterface
 */
public interface GroundInterface extends ModelInterface {

    /**
     * Instructs the robot to make an arcing turn.
     * @param velocity the speed at which to move
     * @param radius the turn radius of the curve
     */
    void sendCurve(int velocity, int radius);

    /**
     * Instructs the robot to move straight forward.
     * @param velocity the speed at which to move
     */
    void sendStraight(int velocity);

    /**
     * Instructs the robot to make a zero-point turn.
     * @param velocity the angular velocity at which to rotate
     * @param angle the amount to rotate
     */
    void sendTurn(int velocity, int angle);

    /**
     * Instructs the robot to stop, cancelling previous instructions.
     */
    void sendReset();
}
