package edu.illinois.mitra.demo.GroupTag;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.illinois.mitra.starl.comms.RobotMessage;
import edu.illinois.mitra.starl.functions.BarrierSynchronizer;
import edu.illinois.mitra.starl.functions.RandomLeaderElection;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LeaderElection;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.interfaces.MessageListener;
import edu.illinois.mitra.starl.interfaces.MutualExclusion;
import edu.illinois.mitra.starl.interfaces.Synchronizer;
import edu.illinois.mitra.starl.models.Model_Ground;
import edu.illinois.mitra.starl.motion.MotionParameters;
import edu.illinois.mitra.starl.motion.MotionParameters.COLAVOID_MODE_TYPE;
import edu.illinois.mitra.starl.motion.RobotMotion;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.PositionList;

public class GroupTagApp extends LogicThread implements MessageListener {
    private static final String TAG = "Logic";
    private static final String ERR = "Critical Error";

    private boolean running = true;
    private RobotMotion motion = null;
    private MutualExclusion mutex = null;

    private boolean iamleader = false;
    private boolean iamfollower = false;
    private LeaderElection le;
    private Synchronizer sync;

    private int tags = 0;// more advanced version will globally update
    private String leader = null;
    private String oldleader = "";
    private ItemPosition targetLocation;
    private String myName = gvh.id.getName();
    private ItemPosition runTo = null;
    private boolean firstRun = true;
    private boolean firstFollow = true;
    private boolean playing =true;
    private boolean goingToMiddle = false;

    private final static int MSG_GAME_OVER=50;
    private final static int MSG_NEWFOLLOWERTAGGED = 25;
    private final static int DIST_THRESHOLD = 230;
    private final static int ANGLE_THRESHOLD = 10;
    private final static int SIDE_THRESHOLD = 200;
    private final static int MID_RADIUS_THRESHOLD = 1000;
    private final static int RESET_RADIUS_THRESHOLD = 25;
    private boolean AT_EDGE = false;
    private int xdistance;
    private int ydistance;
    PositionList posList = new PositionList();

    private final static String SYNC_START = "1";
    private final static String SYNC_BEGINTAG = "2";
    Set<String> toTag = new HashSet<String>(gvh.id.getParticipants());

    final Map<String, ItemPosition> destinations = new HashMap<String, ItemPosition>();
    ItemPosition currentDestination;

    public enum STAGE {
        START, SYNC, LE, SETUP, PLAYING, LEADERMOTION, DONE, SYNC_BEGINTAG
    }

    private STAGE stage = STAGE.START;

    public GroupTagApp(GlobalVarHolder gvh) {
        super(gvh);
 //       gvh.trace.traceStart();
        gvh.log.i(TAG, "I AM " + name);

        sync = new BarrierSynchronizer(gvh);
        le = new RandomLeaderElection(gvh);
        gvh.comms.addMsgListener(this,MSG_NEWFOLLOWERTAGGED);
        gvh.comms.addMsgListener(this,MSG_GAME_OVER);
        motion = gvh.plat.moat;
    }

    @Override
    public List<Object> callStarL() {
        MotionParameters.Builder mp = new MotionParameters.Builder();
        mp = mp.COLAVOID_MODE(COLAVOID_MODE_TYPE.BUMPERCARS);   //buggy
        mp = mp.STOP_AT_DESTINATION(false);
        MotionParameters param = mp.build();
        gvh.plat.moat.setParameters(param);

        while (running) {
            gvh.sleep(100);
            gvh.plat.setDebugInfo(gvh.id.getParticipants().toString());

            switch (stage) {
                case START:
                    sync.barrierSync(SYNC_START);
                    stage = STAGE.SYNC;
                    gvh.log.d(TAG, "Syncing...");
                    //System.out.println("Syncing..." + name);
                    break;
                case SYNC:
                    if (sync.barrierProceed(SYNC_START)) {
                        stage = STAGE.LE;
                        le.elect();
                        gvh.log.d(TAG, "Synced!");
                        //System.out.println("Synced!");
                    }
                    break;
                case LE:
                    if(le.getLeader() != null) {
                        gvh.log.d(TAG, "Electing...");
                        leader = le.getLeader();
                        stage = STAGE.SETUP;
                        iamleader = leader.equals(name);
                        System.out.println("Elected " + leader + " is it me? "+ iamleader);
                    }
                    break;
                case SETUP:
                    if (iamleader) {
                        ItemPosition dest = gvh.gps.getWaypointPosition("middle");
                        System.out.println(dest);
                        gvh.plat.moat.goTo(dest);
                        while (gvh.plat.moat.inMotion)
                        {
                            gvh.sleep(1);
                        }
                        System.out.println("Leader at middle");
                    }
                    sync.barrierSync(SYNC_BEGINTAG);
                    stage = STAGE.SYNC_BEGINTAG;
                    break;
                case SYNC_BEGINTAG:
                    if (sync.barrierProceed(SYNC_BEGINTAG)) {
                        stage = STAGE.PLAYING;
                        //System.out.println("Synced!");
                    }
                    break;
                case DONE:
                    System.out.println(name + " is done.");
                    gvh.plat.moat.motion_stop();
                    return Arrays.asList(results);
                case PLAYING:
                    //System.out.println(stage);
                    if(iamleader){
                        String target = getTarget();
                        targetLocation = gvh.gps.getPosition(target);
                        gvh.plat.moat.goTo(targetLocation);
                        while (gvh.plat.moat.inMotion) {
                            target = getTarget();
                            gvh.sleep(10);
                            if (gvh.plat.moat.inMotion&& ((Model_Ground)gvh.gps.getMyPosition()).angleTo(gvh.gps.getPosition(target)) > ANGLE_THRESHOLD) {
                                break;
                            }
                            if (gvh.gps.getMyPosition().distanceTo( gvh.gps.getPosition(target)) < DIST_THRESHOLD) {
                                tagger(target);
                                break;
                            }
                        }
                    }
                    else if(iamfollower){
                        if(firstFollow){
                            //motion.cancel();
                            gvh.sleep(200);
                            xdistance = gvh.gps.getPosition(myName).getX()-gvh.gps.getPosition(leader).getX();
                            ydistance = gvh.gps.getPosition(myName).getY()-gvh.gps.getPosition(leader).getY();
                            //System.out.println(myName+ "diff: (" + xdistance + "," + ydistance + ")");
                            firstFollow=false;
                        }
                        else{
                            ItemPosition leadPos = gvh.gps.getPosition(leader);
                            int xPart= leadPos.getX();
                            int yPart= leadPos.getY();
                            ItemPosition follPos = new ItemPosition("mypos", xPart+xdistance, yPart+ydistance, 0);
                            //System.out.println(follPos);
                            gvh.plat.moat.goTo(follPos);
                            while(iamfollower && playing){
							/*	Object[] parts = toTag.toArray();
								ItemPosition me = gvh.gps.getMyPosition();
								for (int i = 0; i < parts.length; i++) {
									if(!me.equals(parts[i].toString())){
										if (me.distanceTo(gvh.gps.getPosition(parts[i].toString()))<DIST_THRESHOLD);
										tagger(parts[i].toString());
									}
								}*/
                                leadPos = gvh.gps.getPosition(leader);
                                xPart= leadPos.getX();
                                yPart= leadPos.getY();
                                follPos =new ItemPosition("mypos", xPart+ xdistance, yPart+ ydistance, 0);
                                gvh.plat.moat.goTo(follPos);
                                gvh.sleep(300);
                            }
                        }
                    }
                    else{
                        if(firstRun){
                            runAway();
                            firstRun=false;
                        }
                        else if(gvh.plat.moat.inMotion && !iamleader && goingToMiddle){
                            outHere:
                            while (gvh.plat.moat.inMotion && !iamleader&& goingToMiddle){
                                gvh.sleep(10);
                                //System.out.println(myName+"going to middle");
                                if(gvh.gps.getMyPosition().distanceTo(gvh.gps.getWaypointPosition("middle"))<MID_RADIUS_THRESHOLD){
                                    goingToMiddle=false;
                                    //System.out.println(myName+" should have reset");
                                    runAway();
                                    break outHere;
                                }
                            }
                        }
                        else if (nearedge()){
                            ItemPosition dest = gvh.gps.getWaypointPosition("middle");
                            goingToMiddle=true;
                            gvh.plat.moat.goTo(dest);
                        }
                        else{
                            runAway();
                        }
                        while (gvh.plat.moat.inMotion && !iamleader &&!goingToMiddle) {
                            if(iamfollower)	{
                                break;
                            } else if (nearedge()){
                                ItemPosition dest = gvh.gps.getWaypointPosition("middle");
                                goingToMiddle=true;
                                gvh.plat.moat.goTo(dest);
                            }
                            else if(gvh.gps.getMyPosition().distanceTo(runTo)<RESET_RADIUS_THRESHOLD){
                                runAway();
                            }
                            gvh.sleep(100);
                        }
                    }
            }

        }

        return null;
    }

    @Override
    public void receive(RobotMessage m) {
        switch (m.getMID()) {
            case MSG_NEWFOLLOWERTAGGED:
                if(m.getContents(0).equals(myName))
                    iamfollower=true;
                //System.out.println("iamfollowing = " +myName + " " + iamfollower);
                break;
            case MSG_GAME_OVER:
                stage =STAGE.DONE;
                playing=false;
                break;
        }

    }
    public boolean[] followList(){
        int numBots = gvh.id.getParticipants().size();
        boolean[] Followers = new boolean[numBots];
        for(int i =0; i< numBots-1; i++){
            Followers[i]=false;
        }
        return Followers;
    }

    public boolean nearedge(){
        return ((gvh.gps.getMyPosition().getX() < DIST_THRESHOLD+ gvh.gps.getWaypointPosition("SideA").getX())
                || (gvh.gps.getMyPosition().getX() > gvh.gps.getWaypointPosition("SideB").getX()- DIST_THRESHOLD)
                || (gvh.gps.getMyPosition().getY() < DIST_THRESHOLD+ gvh.gps.getWaypointPosition("Back").getY())
                || (gvh.gps.getMyPosition().getY() > +gvh.gps.getWaypointPosition("Front").getY()- DIST_THRESHOLD));
    }

    public void tagger(String target){
        RobotMessage informNewLeader = new RobotMessage("ALL", name, MSG_NEWFOLLOWERTAGGED, target);
        gvh.comms.addOutgoingMessage(informNewLeader);
        toTag.remove(target);
        System.out.println("Tagged " + target);
        if(toTag.isEmpty()){
            RobotMessage endThisGame = new RobotMessage("ALL", name, MSG_GAME_OVER, "Game Over");
            gvh.comms.addOutgoingMessage(endThisGame);
            stage = STAGE.DONE;
            playing=false;
        }
        tags++;
    }
    public String getTarget(){
        int shortestDistance = Integer.MAX_VALUE;
        String target = null;
        ItemPosition me = gvh.gps.getMyPosition();
        Object[] parts = toTag.toArray();
        for (int i = 0; i < parts.length; i++) {
            if(!me.equals(parts[i].toString())){
                int dist = (int)me.distanceTo(gvh.gps.getPosition(parts[i].toString()));
                if (dist < shortestDistance) {
                    shortestDistance = dist;
                    target =parts[i].toString();
                }
            }
        }
        return target;
    }
    private void runAway(){
        ItemPosition runfrom = gvh.gps.getPosition(leader);
        double angleToGo = (((((Model_Ground)runfrom).angleTo(gvh.gps.getMyPosition()) + 180) % 360));
        //System.out.println("angle to is" +angleToGo );
        int xRun = (int) (Math.cos(angleToGo*Math.PI/180) * 200);
        if(Math.abs((xRun+gvh.gps.getMyPosition().getX()-runfrom.getX())) < Math.abs((-xRun+gvh.gps.getMyPosition().getX()-runfrom.getX()))){
            xRun=-xRun;
        }
        int yRun = (int) (Math.sin(angleToGo*Math.PI/180) * 200);
        if(Math.abs((yRun+gvh.gps.getMyPosition().getY()-runfrom.getY())) < Math.abs((-yRun+gvh.gps.getMyPosition().getY()-runfrom.getY()))){
            yRun=-yRun;
        }
        runTo = new ItemPosition("runTo", gvh.gps.getMyPosition().getX()+xRun, gvh.gps.getMyPosition().getY()+ yRun, 0);
        gvh.plat.moat.goTo(runTo);
    }
}