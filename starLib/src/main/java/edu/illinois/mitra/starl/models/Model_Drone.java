package edu.illinois.mitra.starl.models;

import edu.illinois.mitra.starl.exceptions.ItemFormattingException;
import edu.illinois.mitra.starl.modelinterfaces.DroneInterface;
import edu.illinois.mitra.starl.objects.Common;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.PIDParams;
import edu.illinois.mitra.starl.objects.Point3i;
import edu.illinois.mitra.starl.objects.PositionList;
import edu.illinois.mitra.starl.objects.Vector3f;
import edu.illinois.mitra.starl.objects.Vector3i;

/**
 * This class represents all airborne models. It uses standard flight dynamics parameters
 * yaw, pitch, roll, and gaz, and provides methods predict(), updatePos(), inMotion(), collision(),
 * and updateSensor() used in other classes.
 *
 * Subclasses must override the abstract methods, making sure each returns a constant value.
 *
 * @see Model
 */
public abstract class Model_Drone extends Model {

    // Subclass-specific constants that must be set
    public abstract double max_gaz(); // mm/s 200 to 2000
    public abstract double max_pitch_roll(); // in degrees
    public abstract double max_yaw_speed(); // degrees/s
    public abstract double mass(); // kg
    public abstract double height();

    public abstract Class<? extends DroneInterface> getModelInterface();
    public abstract PIDParams getPIDParams();

    // platform specific control parameters: see page 78 of http://www.msh-tools.com/ardrone/ARDrone_Developer_Guide.pdf
    private double windt;
    private Vector3f wind = new Vector3f(); // mm/s
    private Vector3f windNoise = new Vector3f();

    // Drone control directions
    private double yaw;
    private double pitch;
    private double roll;
    private double gaz;

    // Velocity, translational and rotational
    private Vector3f vel = new Vector3f();
    private double v_yaw;

    private Point3i pos_p = new Point3i();
    private Vector3f vel_p = new Vector3f();
    private double yaw_p;
    private double v_yaw_p;

    private double v_yawR = 0;
    private double pitchR = 0;
    private double gazR = 0;
    private double rollR = 0;

    public Model_Drone() {}

    /**
     * Construct a Model_Drone from a received GPS broadcast message
     *
     * @param received GPS broadcast received
     * @throws ItemFormattingException
     */
    public Model_Drone(String received) throws ItemFormattingException {
        String[] parts = received.replace(",", "").split("\\|");
        if(parts.length == 9) {
            this.name = parts[1];
            this.setPos(Integer.parseInt(parts[2]),
                    Integer.parseInt(parts[3]),
                    Integer.parseInt(parts[4]));
            this.setYaw(Integer.parseInt(parts[5]));
            this.setPitch(Integer.parseInt(parts[6]));
            this.setRoll(Integer.parseInt(parts[7]));
        } else {
            throw new ItemFormattingException("Should be length 9, is length " + parts.length);
        }
    }

    public Model_Drone(String name, int x, int y) {
        super(name, x, y);
    }

    public Model_Drone(String name, int x, int y, int z) {
        super(name, x, y, z);
    }

    public Model_Drone(ItemPosition t_pos) {
        super(t_pos);
    }

    @Override
    public String toString() {
        return name + " (" + getTypeName() + "): " + getPos()
                + "; yaw, pitch, roll, gaz: " + getYaw() + ", " + getPitch() + ", " + getRoll() + " ," + getGaz();
    }

    public Vector3f getVelocity() { return vel; }

    public void setVelocity(Vector3f vel) {
        this.vel = vel;
    }

    /*
	@Override
	public Point3i predict(double[] noises, double timeSinceUpdate) {
		if(noises.length != 3){
			System.out.println("Incorrect number of noises parameters passed in, please pass in getX, getY, getZ, yaw, pitch, roll noises");
			return new Point3i(getX(), getY(), getZ());
		}
		v_yaw += (v_yawR - v_yaw)*timeSinceUpdate;
		pitch += (pitchR - pitch)*timeSinceUpdate;
		roll += (rollR-roll)*timeSinceUpdate;
		gaz += (gazR-gaz)*timeSinceUpdate;

		double xNoise = (getRand()*2*noises[0]) - noises[0];
		double yNoise = (getRand()*2*noises[0]) - noises[0];
		double zNoise = (getRand()*2*noises[0]) - noises[0];
		double yawNoise = (getRand()*2*noises[1]) - noises[1];

		windt += timeSinceUpdate;
		setWindxNoise(xNoise + windx*Math.sin(windt));
		setWindyNoise(yNoise + windy*Math.sin(windt));


		//	double yawNoise = (getRand()*2*noises[3]) - noises[3];
		//double pitchNoise = (getRand()*2*noises[4]) - noises[4];
		//double rollNoise = (getRand()*2*noises[5]) - noises[5];

		//TODO: correct the model

		// speed is in millimeter/second
		// mass in kilograms
		// each pixel is 1 millimeter
		// timeSinceUpdate is in second
		int dX = (int) (xNoise + getV_x() *timeSinceUpdate + getWindxNoise());
		int dY= (int) (yNoise +  getV_y() *timeSinceUpdate + getWindyNoise());
		int dZ= (int) (zNoise +  gaz*timeSinceUpdate);

		x_p = getX() +dX;
		y_p = getY() +dY;
		z_p = getZ() +dZ;

		double thrust;
		if((mass() * Math.cos(Math.toRadians(roll)) * Math.cos(Math.toRadians(pitch))) != 0){
			thrust = ((gaz+1000) / (mass() * Math.cos(Math.toRadians(roll))) / (Math.cos(Math.toRadians(pitch))));
		}
		else{
			thrust = 1000;
		}

		//double thrust = Math.abs((gaz) * (mass * Math.cos(Math.toRadians(roll)) * Math.cos(Math.toRadians(pitch))));
		//double thrust = 100;
		double dv_x = - ((thrust)  * (Math.sin(Math.toRadians(roll)) * Math.sin(Math.toRadians(yaw)) + Math.cos(Math.toRadians(roll)) * Math.sin(Math.toRadians(pitch)) * Math.cos(Math.toRadians(yaw))))/ (mass()) ;
		double dv_y = ((thrust)  * (Math.sin(Math.toRadians(roll)) * Math.cos(Math.toRadians(yaw)) - Math.cos(Math.toRadians(roll)) * Math.sin(Math.toRadians(pitch)) * Math.sin(Math.toRadians(yaw))))/ (mass()) ;


		v_x_p = getV_x() + dv_x * timeSinceUpdate;
		v_y_p = getV_y() + dv_y * timeSinceUpdate;
		v_z_p = gaz;

		double dYaw = (v_yaw*timeSinceUpdate);
		yaw_p = Common.angleWrap(yaw + dYaw);

		return new Point3i(x_p, y_p, z_p);
	}
	*/

    public Point3i predict(double[] noises, double timeSinceUpdate) {
        if(noises.length != 3){
            System.out.println("Incorrect number of noises parameters passed in, please pass in getX, getY, getZ, yaw, pitch, roll noises");
            return new Point3i(getPos());
        }
        v_yaw += (getV_yawR() - v_yaw) * timeSinceUpdate;
        setPitch(getPitch() + (getPitchR() - getPitch()) * timeSinceUpdate);
        setRoll(getRoll() + (getRollR() - getRoll()) * timeSinceUpdate);
        setGaz(getGaz() + (getGazR() - getGaz()) * timeSinceUpdate);

        Vector3f noise = new Vector3f(getRand(), getRand(), getRand())
                .subtract(new Vector3f(0.5, 0.5,  0.5))
                .scale(2 * noises[0]);
        double yawNoise = (getRand() - 0.5) * 2 * noises[1];

        windt += timeSinceUpdate;
        windNoise = new Vector3f(wind).scale(Math.sin(windt));


        //	double yawNoise = (getRand()*2*noises[3]) - noises[3];
        //double pitchNoise = (getRand()*2*noises[4]) - noises[4];
        //double rollNoise = (getRand()*2*noises[5]) - noises[5];

        //TODO: correct the model

        // speed is in millimeter/second
        // mass in kilograms
        // each pixel is 1 millimeter
        // timeSinceUpdate is in second
        Vector3i delta = noise.add(windNoise)
                .add(new Vector3f(vel.getX(), vel.getY(), getGaz()).scale(timeSinceUpdate))
                .toVector3i();

        pos_p = getPos().add(delta);

        double thrust;
        double pitchRad = Math.toRadians(getPitch());
        double rollRad = Math.toRadians(getRoll());
        double yawRad = Math.toRadians(getYaw());
        if (mass() != 0 && Math.cos(rollRad) != 0 && Math.cos(pitchRad) != 0) {
            thrust = (getGaz() +1000) / (mass() * Math.cos(rollRad)) / Math.cos(pitchRad);
        }
        else{
            thrust = 1000;
        }

        Vector3f dv = new Vector3f(-(Math.sin(rollRad) * Math.sin(yawRad) + Math.cos(rollRad) * Math.sin(pitchRad) * Math.cos(yawRad)),
                Math.sin(rollRad) * Math.cos(yawRad) - Math.cos(rollRad) * Math.sin(pitchRad) * Math.sin(yawRad)).scale(thrust / mass());

        vel_p = new Vector3f(vel.getX() + dv.getX() * timeSinceUpdate, vel.getY() + dv.getY() * timeSinceUpdate, getGaz());

        yaw_p = Common.angleWrap(getYaw() + v_yaw * timeSinceUpdate);

        return new Point3i(pos_p);
    }

    @Override
    public final void updatePos(boolean followPredict) {
        if (followPredict) {
            setPos(pos_p);
            setYaw(yaw_p);
            //		pitch = pitch_p;
            //		roll = roll_p;
            v_yaw = v_yaw_p;
            //		v_pitch = v_pitch_p;
            //		v_roll = v_roll_p;
            setVelocity(vel_p);
        } else {
            setPos(getPos().getX(), getPos().getY(), pos_p.getZ());
            vel = new Vector3f(vel.getX(), vel.getY(), vel_p.getZ());
            if(getZ() < 20){
                setRoll(0);
                setPitch(0);
            }
        }
        if(getZ() < 0) {
            setPos(getPos().getX(), getPos().getY(), 0);
            vel = new Vector3f(vel.getX(), vel.getY(), 0);
        }
    }

    @Override
    public final boolean inMotion() {
        return !getVelocity().isZero() || Math.abs(v_yaw) < 1E-6;
    }

    @Override
    public void collision(Point3i collision_point) { //todo(tim) address collision
        // No collision point, set both sensor to false
        if(collision_point != null){
            setGaz(-1000);
        }
    }

    @Override
    public void updateSensor(ObstacleList obspoint_positions,
                             PositionList<ItemPosition> sensepoint_positions) {
        // no sensor model yet
    }

    public final Vector3f getWindNoise() {
        return windNoise;
    }

    public double getYaw() {
        return yaw;
    }

    public void setYaw(double yaw) {
        this.yaw = yaw;
    }

    public double getPitch() {
        return pitch;
    }

    public void setPitch(double pitch) {
        this.pitch = pitch;
    }

    public double getRoll() {
        return roll;
    }

    public void setRoll(double roll) {
        this.roll = roll;
    }

    public double getGaz() {
        return gaz;
    }

    public void setGaz(double gaz) {
        this.gaz = gaz;
    }

    public double getV_yawR() {
        return v_yawR;
    }

    public void setV_yawR(double v_yawR) {
        this.v_yawR = v_yawR;
    }

    public double getPitchR() {
        return pitchR;
    }

    public void setPitchR(double pitchR) {
        this.pitchR = pitchR;
    }

    public double getGazR() {
        return gazR;
    }

    public void setGazR(double gazR) {
        this.gazR = gazR;
    }

    public double getRollR() {
        return rollR;
    }

    public void setRollR(double rollR) {
        this.rollR = rollR;
    }
}
