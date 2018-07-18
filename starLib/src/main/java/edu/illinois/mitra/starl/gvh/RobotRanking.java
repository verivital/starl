package edu.illinois.mitra.starl.gvh;

import android.support.annotation.NonNull;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.illinois.mitra.starl.objects.Common;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.PositionList;

public class RobotRanking {
    private GlobalVarHolder gvh;
    private Map<String,Integer> rankings;

    public RobotRanking(GlobalVarHolder gvh){
        this.gvh = gvh;
        rankings = new HashMap<>();

    }

    public int getRanking(String robID){
        return rankings.get(robID);

    }

/*    public void rankRobots(int leaderNum){
        // All below code in Elect state is to assign order-rank- for each robot in its group

//not leader
        if (robotNum != leaderNum) {
            if (gvh.BotGroup.setAfterBefore) {
                ItemPosition myPosition = gvh.gps.getMyPosition();
                int mySummation = myPosition.getX() + myPosition.getY();
                int ranking = 1;
                PositionList<ItemPosition> plAll = gvh.gps.get_robot_Positions();

//all robots
                for (ItemPosition rp : plAll.getList()) {
                    Integer rpNum = Integer.valueOf(rp.getName().substring(6));
//not leader
                    if (rpNum != leaderNum) {
//all others
                        if (rpNum != robotNum) {
                            Integer rpGroup = Integer.valueOf(rp.getName().substring(6)) % Common.numOFgroups;
//Same group
                            if (gvh.BotGroup.getGroupNum() == rpGroup) {
                                int otherSummation = rp.getX() + rp.getY();
                                // if (mySummation == otherSummation){

 //same summation                               //}
                                if (mySummation == otherSummation)
                                    System.out.println("############************** There is potential same locations ***************########### " + robotName + " and " + rp.getName());
//smaller summation
                                if (mySummation >= otherSummation) {
// should be null first time
// this rob is before bot
                                    if (gvh.BotGroup.BeforeBot == null) {
                                        if (mySummation == otherSummation) {
                                            if (robotNum > Integer.valueOf(rp.getName().substring(6)))
                                                gvh.BotGroup.BeforeBot = rp.getName();
                                        } else
                                            gvh.BotGroup.BeforeBot = rp.getName();
//if before already set
                                    } else {
                                        int xSub = 0;
                                        int ySub = 0;
                                        // int angleSub = 0;
                                        PositionList<ItemPosition> plAllSub = gvh.gps.get_robot_Positions();
//go through all robot positions
//find before bot, set xSub ySub vals
                                        for (ItemPosition rpSub : plAllSub.getList()) {
                                            if (Integer.valueOf(rpSub.getName().substring(6)) == Integer.valueOf(gvh.BotGroup.BeforeBot.substring(6))) {
                                                xSub = rpSub.getX();
                                                ySub = rpSub.getY();
                                                //angleSub = rpSub.angle;
                                            }

                                        }
                                        int beforeBotSummation = xSub + ySub;
//if this summation greater, it becomes beforebot
                                        if (otherSummation > beforeBotSummation)
                                            gvh.BotGroup.BeforeBot = rp.getName();
                                    }
                                    if (mySummation == otherSummation) {

                                        System.out.println("############************** There is potential same locations ***************########### " + robotName + " and " + rp.getName());
                                        if (robotNum < Integer.valueOf(rp.getName().substring(6))) {
                                            gvh.BotGroup.AfterBot = rp.getName();
                                        } else {
                                            gvh.BotGroup.BeforeBot = rp.getName();
                                            ranking++;
                                        }
 //first ranking done, increment
                                    } else
                                        ranking++;
//larger summations
                                } else if (mySummation < otherSummation) {
//set afterbot first time
                                    if (gvh.BotGroup.AfterBot == null)
                                        gvh.BotGroup.AfterBot = rp.getName();
                                    else {
                                        int xSub = 0;
                                        int ySub = 0;
                                        //get cur aftbotsummation
                                        // int angleSub = 0;
                                        PositionList<ItemPosition> plAllSub = gvh.gps.get_robot_Positions();
                                        for (ItemPosition rpSub : plAllSub.getList()) {
                                            if (Integer.valueOf(rpSub.getName().substring(6)) == Integer.valueOf(gvh.BotGroup.AfterBot.substring(6))) {
                                                xSub = rpSub.getX();
                                                ySub = rpSub.getY();
                                                //angleSub = rpSub.angle;
                                            }

                                        }
                                        int afterBotSummation = xSub + ySub;

                                        if (otherSummation == afterBotSummation)
                                            if (robotNum < Integer.valueOf(rp.getName().substring(6)))
                                                gvh.BotGroup.AfterBot = rp.getName();

                                        if (otherSummation < afterBotSummation)
                                            gvh.BotGroup.AfterBot = rp.getName();
                                    }
                                } else if (Integer.valueOf(gvh.id.getName().substring(6)) > Integer.valueOf(rp.getName().substring(6))) {
                                    gvh.BotGroup.BeforeBot = rp.getName();
                                    ranking++;
                                } else
                                    gvh.BotGroup.AfterBot = rp.getName();
                            }


                        }
                    }
                }
//leader|rest
                gvh.BotGroup.rank = ranking;
                gvh.BotGroup.setAfterBefore = false;
//b4 null
                if (gvh.BotGroup.BeforeBot == null) {
                    plAll = gvh.gps.get_robot_Positions();
                    for (ItemPosition rp : plAll.getList())
                        if (Integer.valueOf(rp.getName().substring(6)) == leaderNum)
                            gvh.BotGroup.BeforeBot = rp.getName();
                }
//after null
                if (gvh.BotGroup.AfterBot == null)
                    gvh.BotGroup.isLast = true;

                Common.bots_neighbour[robotNum][0] = gvh.BotGroup.BeforeBot;
                if (!gvh.BotGroup.isLast)
                    Common.bots_neighbour[robotNum][1] = gvh.BotGroup.AfterBot;
                else Common.bots_neighbour[robotNum][1] = "none";
                Common.bots_neighbour[robotNum][2] = String.valueOf(gvh.BotGroup.rf);
                gvh.BotGroup.setAfterBefore = false;
            }
//leader
        } else {
            Common.bots_neighbour[robotNum][0] = "none";
            Common.bots_neighbour[robotNum][1] = "none";
            Common.bots_neighbour[robotNum][2] = "none";
        }

       For Testing purpose
        for (int i=0; i<Common.numOFbots; i++){
            System.out.println("bot"+i+" and his before bot is "+Common.bots_neighbour[i][0]+" and his after bot is "+Common.bots_neighbour[i][1]+" and group distance is "+Common.bots_neighbour[i][2]);
        }

    }*/




}
