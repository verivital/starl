package edu.illinois.mitra.template;

import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.models.ModelRegistry;
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
    //public String bluetooth;
    public Model type;


    public BotInfoSelector(String color, int type, int deviceType) {
        switch (color) {
            case "red":
                name = "bot0"; // assign name: bot0 is always red
                switch (deviceType) {
                    case Common.NEXUS7:
                        //ip = "192.168.1.110"; // reserved IP address of red Nexus7 tablet
                        ip = "10.255.24.203";
                        break;
                    case Common.MOTOE:
                        //ip = "192.168.1.114"; // reserved IP address of red MotoE phone
                        ip = "10.255.24.114";
                        break;
                }
                switch (type) {
                    case Common.IROBOT:
                        this.type = ModelRegistry.create("Model_iRobot", name, 0, 0);
                        break;
                    case Common.MINIDRONE:
                        this.type = ModelRegistry.create("Model_quadcopter", name, 0, 0);
                        break;
                    case Common.MAVIC:
                        this.type = ModelRegistry.create("Model_Mavic", name, 0, 0);
                        break;
                    case Common.o3DR:
                        this.type = ModelRegistry.create("Model_3DR", name, 0, 0);
                        break;
                    case Common.PHANTOM:
                        this.type = ModelRegistry.create("Model_Phantom", name, 0, 0);
                        break;
                    case Common.GHOSTAERIAL:
                        this.type = ModelRegistry.create("Model_GhostAerial", name, 0, 0);
                        break;
                }
                break;

            case "green":
                name = "bot1";
                switch (deviceType) {
                    case Common.NEXUS7:
                        //ip = "192.168.1.111";
                        ip = "10.255.24.111";
                        break;
                    case Common.MOTOE:
                        //ip = "192.168.1.115";
                        ip = "10.255.24.115";
                        break;
                }
                switch (type) {
                    case Common.IROBOT:
                        this.type = ModelRegistry.create("Model_iRobot", name, 0, 0);
                        break;
                    case Common.MINIDRONE:
                        this.type = ModelRegistry.create("Model_quadcopter", name, 0, 0);
                        break;
                    case Common.GHOSTAERIAL:
                        this.type = ModelRegistry.create("Model_GhostAerial", name, 0, 0);
                        break;
                }
                break;

            case "blue":
                name = "bot2";
                //ip = "192.168.1.112";
                ip = "10.255.24.152";
                switch (type) {
                    case Common.IROBOT:
                        this.type = ModelRegistry.create("Model_iRobot", name, 0, 0);
                        break;
                    case Common.MINIDRONE:
                        this.type = ModelRegistry.create("Model_quadcopter", name, 0, 0);
                        break;
                    case Common.PHANTOM:
                        this.type = ModelRegistry.create("Model_Phantom", name, 0, 0);
                        break;
                    case Common.GHOSTAERIAL:
                        this.type = ModelRegistry.create("Model_GhostAerial", name, 0, 0);
                        break;
                }
                break;

            case "white":
                name = "bot3";
                //ip = "192.168.1.113";
                ip = "10.255.24.113";
                switch (type) {
                    case Common.IROBOT:
                        this.type = ModelRegistry.create("Model_iRobot", name, 0, 0);
                        break;
                    case Common.MINIDRONE:
                        break;
                    case Common.GHOSTAERIAL:
                        this.type = ModelRegistry.create("Model_GhostAerial", name, 0, 0);
                        break;
                }
                break;
        }
    }
}
