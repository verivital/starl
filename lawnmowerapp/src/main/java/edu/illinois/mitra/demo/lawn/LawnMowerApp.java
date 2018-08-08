package edu.illinois.mitra.demo.lawn;

import java.util.HashMap;
import java.util.List;
import java.util.Map;


import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.motion.RobotMotion;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starlSim.main.SimSettings;

public class LawnMowerApp extends LogicThread {
    int shift_amount=0;
    private enum STAGE { START, MOVE, MOVE_DOWN,MOVE_LEFT,MOVE_UP,MOVE_RIGHT,MOVE_BACK, DONE }
    private STAGE stage = STAGE.START;
    private static final int WAYPOINTS_TO_FOLLOW = 9;

    private String robotName;
    private int robotNum;


    private int cur_waypoint = 0;
    private SimSettings.Builder settings = new SimSettings.Builder();

    final Map<String, ItemPosition> destinations = new HashMap<String, ItemPosition>();
    ItemPosition currentDestination;


    public LawnMowerApp(GlobalVarHolder gvh) {
        super(gvh);
//        gvh.trace.traceStart();

        robotName = gvh.id.getName();
        robotNum = gvh.id.getIdNumber();
    }

    @Override
    public List<Object> callStarL() {
        while(true) {
            switch (stage) {
                case START:
                    gvh.trace.traceSync("LAUNCH",gvh.time());
                    stage = STAGE.MOVE;
                    gvh.plat.moat.goTo(startpoint());
                    break;

                case MOVE:
                    if(!gvh.plat.moat.inMotion) {
                        if(cur_waypoint < WAYPOINTS_TO_FOLLOW) {
                            cur_waypoint ++;
                            gvh.plat.moat.goTo(startpoint());
                        } else {
                            stage = STAGE.MOVE_DOWN;
                        }
                    }
                    break;
                case MOVE_DOWN:
                    gvh.plat.moat.goTo(movedown());
                    while(gvh.plat.moat.inMotion) {
                        gvh.sleep(10);}
                    stage=STAGE.MOVE_LEFT;
                    break;
                case MOVE_LEFT:
                    gvh.plat.moat.goTo(moveleft());
                    while(gvh.plat.moat.inMotion) {
                        gvh.sleep(10);}
                    stage=STAGE.MOVE_UP;
                    break;
                case MOVE_UP:
                    gvh.plat.moat.goTo(moveup());
                    while(gvh.plat.moat.inMotion) {
                        gvh.sleep(10);}
                    stage=STAGE.MOVE_RIGHT;
                    break;
                case MOVE_RIGHT:
                    gvh.plat.moat.goTo(moveright());
                    while(gvh.plat.moat.inMotion) {
                        gvh.sleep(10);}
                    stage=STAGE.MOVE_BACK;
                    break;

                case MOVE_BACK:
                    gvh.plat.moat.goTo(startpoint());
                    while(gvh.plat.moat.inMotion) {
                        gvh.sleep(10);}
                    stage=STAGE.DONE;
                    break;

                case DONE:
                    System.out.println("Done");
                    gvh.trace.traceEnd();
                    return returnResults();
            }
            gvh.sleep(100);
        }//TODO: Add a shift stage that moves the robots over so that the spaces between the lengths of the robots
        //are still cut. This offset would entail a second trip around the area
    }


    private ItemPosition startpoint() {
        return new ItemPosition("goHere",settings.getGRID_XSIZE()/2 + robotNum * 500, settings.getGRID_YSIZE()/2, 0);
    }
    private ItemPosition movedown() {
        return new ItemPosition("goHere",settings.getGRID_XSIZE()/2 + robotNum * 500, (int) (settings.getGRID_YSIZE()/2 - robotNum*300*1.414), 0);
    }
    private ItemPosition moveleft() {
        return new ItemPosition("goHere",(int) (settings.getGRID_XSIZE()/2 - robotNum * 300*1.414), (int) (settings.getGRID_YSIZE()/2 - robotNum*300*1.414), 0);
    }
    private ItemPosition moveup() {
        return new ItemPosition("goHere",(int) (settings.getGRID_XSIZE()/2 - robotNum * 300*1.414), (int) (settings.getGRID_YSIZE()/2 + robotNum*300*1.414), 0);
    }
    private ItemPosition moveright() {
        return new ItemPosition("goHere",(int) (settings.getGRID_XSIZE()/2 + robotNum * 300*1.414), (int) (settings.getGRID_YSIZE()/2 + robotNum*300*1.414), 0);
    }
}
