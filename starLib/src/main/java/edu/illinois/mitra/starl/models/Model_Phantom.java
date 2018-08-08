package edu.illinois.mitra.starl.models;

import edu.illinois.mitra.starl.exceptions.ItemFormattingException;
import edu.illinois.mitra.starl.modelinterfaces.DjiController;
import edu.illinois.mitra.starl.modelinterfaces.DroneInterface;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.PIDParams;

/**
 * A model of the DJI Phantom.
 * @see Model_Drone
 */

public class Model_Phantom extends Model_Drone {

	@SuppressWarnings("unused")
	public Model_Phantom() {}

	@SuppressWarnings("unused")
	public Model_Phantom(String received) throws ItemFormattingException {
		super(received);
	}

	@SuppressWarnings("unused")
	public Model_Phantom(String name, int x, int y) {
		super(name, x, y);
	}

	@SuppressWarnings("unused")
	public Model_Phantom(String name, int x, int y, int z) {
		super(name, x, y, z);
	}

	@SuppressWarnings("unused")
	public Model_Phantom(ItemPosition t_pos) {
		super(t_pos);
	}

	@Override
	public int radius() { return 340; }

	@Override
	public String ip() { return "10.1.1.10"; }

	@Override
	public double height() { return 50; }

	@Override
	public double mass() { return 1.216; }	// Phantom 3 mass with battery + propellers is 1216g

	@Override
	public double max_gaz() { return 1000; }

	@Override
	public double max_pitch_roll(){
		//return 20;
		return 40;
	}

	@Override
	public double max_yaw_speed() { return 200; }

	@Override
	public Class<? extends DroneInterface> getModelInterface() {
		return DjiController.class;
	}

	@Override
	public PIDParams getPIDParams() {
		PIDParams p = new PIDParams();
		//p.Kp = 0.0714669809792096;
		//p.Ki = 0.0110786899216426;
		//p.Kd = 0.189205037832174;
		p.Kp = 3E-4;
		p.Ki = 0.0;
		p.Kd = 6E-4;
		p.saturationLimit = 0;//Math.sin(max_pitch_roll());
		p.windUpLimit = 0;
		p.filterLength = 10;
		p.reversed = true;
		return p;
	}
}
