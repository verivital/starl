package edu.illinois.mitra.starl.models;

import edu.illinois.mitra.starl.exceptions.ItemFormattingException;
import edu.illinois.mitra.starl.motion.GroundBTI;
import edu.illinois.mitra.starl.motion.IRobotBTI;
import edu.illinois.mitra.starl.objects.ItemPosition;

/**
 * This class represents a simple model of the iRobot Create, including angle, radius, type, velocity, leftBump, rightBump, circleSensor, vFwd, vRad
 * and some prediction on getX and getY based on vFwd and vRad
 *
 * @author Yixiao Lin
 * @version 1.0
 */
public class Model_iRobot extends Model_Ground {

	//private boolean circleSensor = false;

	static {
		ModelRegistry.register(Model_iRobot.class);
	}

	/**
	 * Construct an Model_iRobot from a received GPS broadcast message
	 *
	 * @param received GPS broadcast received
	 * @throws ItemFormattingException
	 */

	public Model_iRobot(String received) throws ItemFormattingException{
		String[] parts = received.replace(",", "").split("\\|");
		if(parts.length == 7) {
			this.name = parts[1];
			this.setPos(Integer.parseInt(parts[2]),
					Integer.parseInt(parts[3]),
					Integer.parseInt(parts[4]));
			this.angle = Integer.parseInt(parts[5]);
//		} else {
//			throw new ItemFormattingException("Should be length 7, is length " + parts.length);
//        String[] parts = received.replace(",", "").split("\\|");
//        if(parts.length == 6) {
//            this.angle = Double.parseDouble(parts[4]);
		}
        else {
            throw new ItemFormattingException("Should be length 5, is length " + parts.length);
        }
	}

	public Model_iRobot(String name, int x, int y) {
		super(name, x, y);
	}

	public Model_iRobot(String name, int x, int y, double angle) {
		super(name, x, y);
		this.angle = angle;
	}

	public Model_iRobot(ItemPosition t_pos) {
		super(t_pos);
		// TODO Auto-generated constructor stub
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
