/**
 * Classes specific to each kind of robot.
 *
 * This package contains classes holding all of the information
 * specific to each kind of robot that StarL can interface with
 * and simulate. Most of the work required to integrate a new
 * robot into StarL is done here, without having to modify any
 * external classes.
 * <p>
 * {@link edu.illinois.mitra.starl.models.Model} is the base class
 * for all types of robots, and is used widely to represent the
 * "current robot." It provides abstract methods for characteristics
 * that all types of robots share, including a method to return
 * the type of physical interface to be used to control each robot.
 * <p>
 * {@link edu.illinois.mitra.starl.models.Model_Ground} is the base
 * class for non-aerial robots that have zero-point turning, i.e.
 * a forward velocity and an angle.
 * <p>
 * {@link edu.illinois.mitra.starl.models.Model_Drone} is the base
 * class for aerial robots that have yaw, pitch, roll, and gaz
 * (vertical velocity).
 * <p>
 * {@link edu.illinois.mitra.starl.models.ModelRegistry} is a
 * container class with static methods used to register each non-
 * abstract subclass of Model for use in the rest of StarL.
 * <p>
 * To add a new kind of robot, you must do the following:
 * <ol>
 *     <li>Create a new class representing the robot (<code>Model_YourClass</code>),
 *     subclassing the appropriate base class.</li>
 *     <li>Implement all abstract methods.</li>
 *     <li>Add a call to <code>register(Model_YourClass.class)</code> in
 *     {@link edu.illinois.mitra.starl.models.ModelRegistry}.</li>
 *     <li>If no existing ModelInterface is compatible with the new Model, implement an interface
 *     that is compatible. Not necessary for simulation.</li>
 *     <li>That's it! StarL is flexible enough to be ready to use your class in applications.</li>
 * </ol>
 */
package edu.illinois.mitra.starl.models;
