package edu.illinois.mitra.starl.modelinterfaces;

/**
 * The base interface of any interface for communicating with real robots.
 */
public interface ModelInterface {
    /**
     * Disconnect from the robot, finalizing any ongoing instructions.
     */
    void disconnect();
}

// TODO make sure all ModelInterfaces have default constructor or get their correct dependencies