# StarL 1.5.1

Find the documentation [here](https://verivital-starl-undergrads.github.io/starl/).

https://wiki.cites.illinois.edu/wiki/display/MitraResearch/StarL

https://www.verivital.slack.com

http://www.verivital.com/

http://www.isis.vanderbilt.edu/


## Quick Start Guide
### Initiating the App Instance
In order to run your StarL app, first open `RobotsActivity.java` in starlTemplate and find the following code:

```java
runThread = new FollowApp(gvh);
```

Change this line so that you instantiate your app instead of FollowApp. You will have to import your app and also create the dependency
in the `build.gradle` of starlTemplate.

```gradle
dependencies {
    ...
    implementation project(':followApp')
}
```
### Setting the Tablet and Drone Info
Open `BotInfoSelector.java` and set the name and IP address for each colored
robot. For example:
```java
...
case "blue":
    name = "bot2"; // this name has to match the one in MatLab for the color
    ip = "10.255.24.152"; // ip address for the mobile device
    switch (type) {
        case Common.IROBOT:
            this.type = ModelRegistry.create("Model_iRobot", name, 0, 0);
            break;
        case Common.MINIDRONE:
            this.type = ModelRegistry.create("Model_quadcopter", name, 0, 0);
            break;
        case Common.PHANTOM:
            this.type = ModelRegistry.create("Model_Phantom", name, 0, 0);
            break;
        case Common.GHOSTAERIAL:
            this.type = ModelRegistry.create("Model_GhostAerial", name, 0, 0);
            break;
    }
    break;
...
```
Then back in `RobotsActivity.java` in the `loadBotInfo()` method you can create each robot info 
with the corresponding color:
```java
return new BotInfoSelector[] {
        new BotInfoSelector("blue", "Model_Phantom", Common.NEXUS7)
        //, other entries
};
```

### Configuring the MatLab "GPS"
In `UdpGpsReceiver.java` you must set the incoming robot position data to be the right type of drone.
```java
case '$':
    try{
        Model newpos = ModelRegistry.create("Model_Phantom", part);
        ...
    }
```
In this case, all robot position data will be read as Phantom drone data.

### Launching the App
Connect a mobile device and press the green arrow. If the app crashes, make sure to enable permissions on the mobile device
for the app.

### Launching the Drone(s)
If the Bluetooth and Wi-Fi addresses have been correctly entered, or if the DJI remote has been been correctly
connected, the application will automatically connect to the robot. The robot name can be tapped in order to switch
the robot which the application is connecting to. **No other part of the UI is interactable. The checkboxes are status
indicators.** 

In the case that the GPS connects first followed by the drone(s), a launch command will have to be sent via MatLab.

In the case that the GPS connects after the drone has connected, the drone(s) will automatically launch.

### Common Reasons for Crashing or not Compiling
- The proper permissions are not enabled - *crash on application launch.*
- MatLab lost track of the drone - *crash while the drone is running.*
- The MatLab drone name for a certain color robot is different - *crash on connection.*
- Instant Run is enabled. - *compile error*
- The project you opened is `starlTemplate` instead of `starl`. - *compile error regarding Mavlink*
- You do not have the correct build tools or Android libraries installed. Use the SDK manager to fix this. - *compile error*
- If you have been running simulations, the `Java Platform SE Binary` task may have to be stopped. - *compile error*

## Running Simulations
Run `Main.java` for the StarL application that you want. `Main.java` also contains the settings for the simulation. In order for the simulation to run, the run configurations
must have the `classpath of module` to be set to the appropriate StarL application. In order for the waypoints to be loaded,
the working directory should be set to be the directory containing the waypoints folder.

*Note:* The waypoint folder handles waypoints for the simulation. Real life waypoints are set in MatLab.

## Creating a New StarL Application
The `starLTemplate` Android application has been designed as the interface for all StarL applications. It is not recommended
to write a new Android application. Instead, new StarL applications should be written as 
a module which can be instantiated with `starlTemplate`.

Please use the provided applications as a guideline. Essentially, there should be one class which handles the state changes
involved in controlling a drone, one class which handles drawing, and one class which contains the settings
and runs the simulation.

It is important for every new StarL application to have the appropriate dependencies. The `build.gradle` should contain the following:
```gradle
dependencies {
    implementation project(':starLSim')
    implementation project(':starLib')
}
```
## Features to be Implemented
- Ability to accept robot position data for multiple types of robots
- Automatic permission requesting
- Better crash avoidance calculation
- Reduce time required to launch simulation
- Clarify some variables since some drones do not use Bluetooth

## New Drone Statuses
### DJI SDK - STATUS: PID needs more tuning
@timliang @stirling

Compatible with Mavic and Phantom DJI drones.
The tablet must be connected to the remote controller of the drone via USB or [Wi-Fi Bridge App](https://github.com/dji-sdk/Android-Bridge-App). Wi-Fi bridging can be toggled inside `DjiController.java`.
```java
private static final boolean USING_WIFI_BRIDGE = false;
```
See the "Setting the Tablet and Drone Info" section for how to change Wi-Fi bridge IP address. **Do not set the IP to
that of the device running StarL.**

*Note:* When tuning the PID controller, it is important to remember that the controller was designed for
the AR Minidrone which has opposite sign conventions for pitch as compared to DJI. Also,
the DJI pitch and roll may be flipped.

### Ehang SDK - STATUS: Partially Working
@christina @ehrlichwirklich

The PID Controller has not been tuned, and the drone has trouble taking off.

### 3DR SDK - STATUS: Untested
@austinwilms @yangy14

The 3DR Solo requires a GPS satellite connection and could not be run indoors via MatLab navigation.

---
For further documentation or implementation clarification regarding the above drones, please message the corresponding authors at https://www.verivital.slack.com.

See the `Documentation` folder for other materials on the framework.

