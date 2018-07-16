package edu.illinois.mitra.starl.models;

import edu.illinois.mitra.starl.exceptions.ItemFormattingException;
import edu.illinois.mitra.starl.motion.GroundBTI;
import edu.illinois.mitra.starl.motion.IRobotBTI;
import edu.illinois.mitra.starl.objects.ItemPosition;

/**
 * This class represents a simple model of the iRobot Create, including angle, radius, type, velocity, leftBump, rightBump, circleSensor, vFwd, vRad
 * and some prediction on x and y based on vFwd and vRad
 *
 * @author Yixiao Lin
 * @version 1.0
 */
public class Model_iRobot extends Model_Ground {

	static {
		ModelRegistry.register(Model_iRobot.class);
	}

	public Model_iRobot(String received) throws ItemFormattingException{
		super(received);
	}

	public Model_iRobot(String name, int x, int y) {
		super(name, x, y);
	}

	public Model_iRobot(String name, int x, int y, double angle) {
		super(name, x, y, angle);
	}

	public Model_iRobot(ItemPosition t_pos) {
		super(t_pos);
	}

	@Override
	public int radius() {
	    return 165;
	}

	@Override
	public Class<? extends GroundBTI> getBluetoothInterface() {
		return IRobotBTI.class;
	}
}
