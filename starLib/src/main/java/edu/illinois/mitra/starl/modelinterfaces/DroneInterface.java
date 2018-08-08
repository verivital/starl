package edu.illinois.mitra.starl.modelinterfaces;

/**
 * The interface for communicating with aerial drone robots.
 *
 * @see ModelInterface
 */
public interface DroneInterface extends ModelInterface {

    /**
     * Sends the yaw, pitch, roll, and gaz values for the drone to match.
     * @param yaw the rotation about the vertical axis, in the range [-1, 1]
     * @param pitch the angle of the nose of the drone, in the range [-1, 1]
     * @param roll the angle of the left and right sides of the drone, in the range [-1, 1]
     * @param gaz the vertical velocity of the drone, in the range [-1, 1]
     */
    void setControlInput(double yaw, double pitch, double roll, double gaz);

    /**
     * Instructs the drone to take off.
     */
    void sendTakeoff();

    /**
     * Instructs the drone to land.
     */
    void sendLanding();

    /**
     * Instructs the drone to perform emergency recovery (often the same as {@link #sendLanding()}).
     */
    void sendEmergency();

    /**
     * Instructs the drone to stop and hover in midair.
     */
    void hover();

    /**
     * Sets the maximum angle that the drone should tilt to.
     * @param maxTilt the angle, in <b>degrees</b>
     */
    void setMaxTilt(float maxTilt);
}
