package edu.illinois.mitra.starl.models;

import edu.illinois.mitra.starl.exceptions.ItemFormattingException;
import edu.illinois.mitra.starl.modelinterfaces.DjiController;
import edu.illinois.mitra.starl.modelinterfaces.DroneInterface;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.PIDParams;

/**
 * A model of the DJI Mavic.
 * @see Model_Drone
 */
public class Model_Mavic extends Model_Drone {

	@SuppressWarnings("unused")
	public Model_Mavic() {}

	@SuppressWarnings("unused")
	public Model_Mavic(String received) throws ItemFormattingException {
		super(received);
	}

	@SuppressWarnings("unused")
	public Model_Mavic(String name, int x, int y) {
		super(name, x, y);
	}

	@SuppressWarnings("unused")
	public Model_Mavic(String name, int x, int y, int z) {
		super(name, x, y, z);
	}

	@SuppressWarnings("unused")
	public Model_Mavic(ItemPosition t_pos) {
		super(t_pos);
	}

	@Override
	public int radius() { return 340; }

	@Override
	public String ip() { return "10.1.1.10"; }

	@Override
	public double height() { return 50; }

	@Override
	public double mass() {
		// Mavic drone mass is 734g with battery without gimbal cover | 743g with cover
		return .734;
	}

	@Override
	public double max_gaz() { return 1000; }

	@Override
	public double max_pitch_roll() { return 20; }

	@Override
	public double max_yaw_speed() { return 200; }

	@Override
	public Class<? extends DroneInterface> getModelInterface() {
		return DjiController.class;
	}

	@Override
	public PIDParams getPIDParams() {
		PIDParams p = new PIDParams();
		p.Kp = 3.25E-4;//0.0714669809792096;
		p.Ki = 0.75*p.Kp;//0.0110786899216426;
		p.Kd = 1.75E-3;//0.113205037832174;
		p.saturationLimit = 50;
		p.windUpLimit = 185;
		p.filterLength = 8;
		p.reversed = true;
		return p;
	}
}
