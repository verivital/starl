package edu.illinois.mitra.starl.gvh;

import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.motion.ReachAvoid;
import edu.illinois.mitra.starl.motion.RobotMotion;

/**
 * Stub class implementing platform specific methods.
 * 
 * @author Adam Zimmerman
 * @version 1.0
 *
 */
public class AndroidPlatform {
	
	public ReachAvoid reachAvoid;
	
	public RobotMotion moat;
	
	public Model model;
		
    public void setDebugInfo(String debugInfo) {
	}
	
	public void sendMainToast(String debugInfo) {
	}
	
	public void sendMainMsg(int type, Object data) {
	}
	
	public void sendMainMsg(int type, int arg1, int arg2) {		
	}

	public Model getModel() {
		return model;
	}
}
