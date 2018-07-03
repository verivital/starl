package edu.illinois.mitra.template;

import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.models.Model_GhostAerial;
import edu.illinois.mitra.starl.models.Model_Mavic;
import edu.illinois.mitra.starl.models.Model_Phantom;
import edu.illinois.mitra.starl.models.Model_iRobot;
import edu.illinois.mitra.starl.models.Model_quadcopter;
import edu.illinois.mitra.starl.models.Model_3DR;
import edu.illinois.mitra.starl.objects.Common;

/**
 * Created by VerivitalLab on 3/9/2016.
 * This class contains all info for specific hardware addresses
 * Each tablet/phone and robot is assigned a color
 * This files specifies the phone/tablets' IP addresses, robot's names and bluetooth address
 * All addresses and names are based on the color, as you can see in the if statements below
 */
public class BotInfoSelector {

    public String name;
    public String ip;
    public String bluetooth;
    public Model type;


    public BotInfoSelector(String color, int type, int deviceType) {
        if(color.equals("red")) {
            name = "bot0"; // assign name: bot0 is always red
            if(deviceType == Common.NEXUS7) {
                //ip = "192.168.1.110"; // reserved IP address of red Nexus7 tablet
                ip = "10.255.24.203";
            }
            else if(deviceType == Common.MOTOE) {
                //ip = "192.168.1.114"; // reserved IP address of red MotoE phone
                ip = "10.255.24.114";
            }
            if(type == Common.IROBOT) {
                bluetooth = "5C:F3:70:75:BB:2F"; // bluetooth address of red raspberry pi on red irobot
                this.type = new Model_iRobot(name, 0,0);
            }
            else if(type == Common.MINIDRONE) {
                bluetooth = "Mars_122139"; // bluetooth address of red minidrone (use free flight 3 app from play store to get this address)
                this.type = new Model_quadcopter(name, 0,0);
            }
            else if(type == Common.MAVIC){
                bluetooth = "Mavic uses USB";
                this.type = new Model_Mavic(name, 0, 0);
            }

            else if(type == Common.o3DR){
                bluetooth = "o3DR USES WIFI";
                this.type = new Model_3DR(name, 0, 0);
            }

            else if(type == Common.PHANTOM){
                bluetooth = "Phantom uses USB";
                this.type = new Model_Phantom(name, 0, 0);
            }
            else if(type == Common.GHOSTAERIAL) {
                bluetooth = "98:D3:32:20:58:5B"; // bluetooth address of GBOX
                this.type = new Model_GhostAerial(name, 0,0);
            }
        }

        if(color.equals("green")) {
            name = "bot1";
            if(deviceType == Common.NEXUS7) {
                //ip = "192.168.1.111";
                ip = "10.255.24.111";
            }
            else if(deviceType == Common.MOTOE) {
                //ip = "192.168.1.115";
                ip = "10.255.24.115";
            }
            if(type == Common.IROBOT) {
                bluetooth = "5C:F3:70:75:BB:2F";
                this.type = new Model_iRobot(name, 0,0);
            }
            else if(type == Common.MINIDRONE) {
                bluetooth = "green1";
                this.type = new Model_quadcopter(name, 0,0);
            }
            else if(type == Common.GHOSTAERIAL) {
                bluetooth = "98:D3:32:20:58:5B"; // bluetooth address of GBOX
                this.type = new Model_GhostAerial(name, 0,0);
            }
        }

        if(color.equals("blue")) {
            name = "bot2";
            //ip = "192.168.1.112";
            ip = "10.255.24.152";
            if(type == Common.IROBOT) {
                bluetooth = "5C:F3:70:76:CE:B4";
                this.type = new Model_iRobot(name, 0,0);
            }
            else if(type == Common.MINIDRONE) {
                bluetooth = "Travis_064729";
                this.type = new Model_quadcopter(name, 0,0);
            }
            else if(type == Common.PHANTOM){
                bluetooth = "NA";
                this.type = new Model_Phantom(name, 0, 0);
            }
            else if(type == Common.GHOSTAERIAL) {
                bluetooth = "98:D3:32:20:58:5B"; // bluetooth address of GBOX
                this.type = new Model_GhostAerial(name, 0,0);
            }
        }

        if(color.equals("white")) {
            name = "bot3";
            //ip = "192.168.1.113";
            ip = "10.255.24.113";
            if(type == Common.IROBOT) {
                bluetooth = "5C:F3:70:76:C9:D4";
                this.type = new Model_iRobot(name, 0,0);
            }
            else if(type == Common.MINIDRONE) {
                //bluetooth = ""; There isn't a white drone set-up yet
            }
            else if(type == Common.GHOSTAERIAL) {
                bluetooth = "98:D3:32:20:58:5B"; // bluetooth address of GBOX
                this.type = new Model_GhostAerial(name, 0,0);
            }
        }

    }
}
