package edu.illinois.mitra.starl.motion;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.modelinterfaces.GroundInterface;
import edu.illinois.mitra.starl.motion.MotionAutomaton_Ground;
import edu.illinois.mitra.starl.objects.Common;

/**
 * Subclasses MotionAutomaton_Ground to provide support for real ground robots.
 */
public class RealMotionAutomaton_Ground extends MotionAutomaton_Ground {
    private final GroundInterface bti;

    public RealMotionAutomaton_Ground(GlobalVarHolder gvh, GroundInterface bti) {
        super(gvh);
        this.bti = bti;
    }

    @Override
    public void motion_stop() {
        straight(0);
        stage = STAGE.INIT;
        this.destination = null;
        running = false;
        inMotion = false;
    }

    @Override
    protected void curve(int velocity, int radius) {
        if(running) {
            sendMotionEvent(Common.MOT_ARCING, velocity, radius);
            bti.sendCurve(velocity, radius);
        }
    }

    @Override
    protected void straight(int velocity) {
        gvh.log.i(TAG, "Straight at velocity " + velocity);
        if(running) {
            if(velocity != 0) {
                sendMotionEvent(Common.MOT_STRAIGHT, velocity);
            } else {
                sendMotionEvent(Common.MOT_STOPPED, 0);
            }
            System.out.println("straight");
            bti.sendStraight(velocity);
        }
    }

    @Override
    protected void turn(int velocity, int angle) {
        if(running) {
            sendMotionEvent(Common.MOT_TURNING, velocity, angle);
            System.out.println("turn");
            bti.sendTurn(velocity, angle);
        }
    }

    @Override
    public void cancel() {
        running = false;
        bti.sendReset();
        bti.disconnect();
    }
}
