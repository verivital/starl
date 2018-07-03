package edu.illinois.mitra.starl.models;

import edu.illinois.mitra.starl.objects.ItemPosition;

/**
 *TODO: Merge with josh's if more indepth.
 *TODO: Make sure all other models extend this model, make sure they modify these parameters rather than declaring their own.
 */

//
public abstract class Model_Drone extends Model {


  // Subclass-specific constants that must be set
  public abstract double max_gaz();   // millimeter/s 200 to 2000
  public abstract double max_pitch_roll();  // in degrees
  public abstract double max_yaw_speed(); // degrees/s
  public abstract double mass();     // mass in kilograms
  public abstract double height();

    //Drone control directions
    public double yaw;
    public double pitch;
    public double roll;
    public double gaz;

    //Used for inMotion Method, update position
    public double v_x;
    public double v_y;
    public double v_z;
    public double v_yaw;

    public double v_yawR = 0;
    public double pitchR = 0;
    public double gazR = 0;
    public double rollR = 0;

    public Model_Drone(){

    }
    public Model_Drone(String name, int x, int y){
        super(name, x, y);
    }

    public Model_Drone(String name, int x, int y, int z) {
        super(name, x, y, z);
    }

    public Model_Drone(String name, int x, int y, int z, double yaw, double pitch, double roll, int radius) {
        super(name, x, y, z);
    }

    public Model_Drone(String name, int x, int y, int z, int yaw) {
        super(name, x, y, z);
    }


    public Model_Drone(ItemPosition t_pos) {
        super(t_pos.name, t_pos.x, t_pos.y, t_pos.z);
    }


}
