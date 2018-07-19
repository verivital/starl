/**
 * Created by Mousa Almotairi on 4/28/2015.
 */

package edu.illinois.mitra.starl.gvh;

import java.util.ArrayList;

import edu.illinois.mitra.starl.objects.Common;


public class RobotGroup {

    Integer groupNum;
    public Double theta;
    ArrayList<String> botLists;
    public boolean setAfterBefore;
    public String AfterBot;
    public String BeforeBot;
    public int rank;
    public int rf;
    public boolean isLast;


    public RobotGroup(String id, Integer numOFgroup){

        String intValue = id.replaceFirst("[^0-9]+", ""); // this will work for bots with sequential numbers in their names, not irobot0 quadrotor0
        Integer i = Integer.parseInt(intValue);
        groupNum = i % numOFgroup;
        setAfterBefore= true;
        rank = 0;
        isLast= false;

        double calcuateAngle = groupNum*(360/Common.numOFgroups);
        if (Common.numOFgroups == 2 && groupNum ==1){
            theta = Double.valueOf(90);

        }
        else{
            theta = calcuateAngle;

        }
        // rf = 500* (groupNum+1);

        rf = 1000;  //Has problems with lower values


        System.out.println("This robot is "+id+ " and it is assigned to group number "+ getGroupNum().toString()+" and ts theta is "+ theta.toString());
    }

    public Integer getGroupNum(){
        return groupNum;
    }
}
