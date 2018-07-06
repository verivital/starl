package edu.illinois.mitra.starl.interfaces;

/**
 * Interface used to receive a communicate which key was pressed by passing a string.
 * Implemented in RobotMotion for use of a user interface.
 */
public interface AcceptsKeyInput {

    /**
     * Used to communicate which key was pressed and when it is released.
     * @param key -- String representing which key was pressed. Changed to "stop" when released.
     */
    public void receivedKeyInput(String key);
}
