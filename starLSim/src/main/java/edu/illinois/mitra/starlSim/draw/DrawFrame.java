package edu.illinois.mitra.starlSim.draw;

import java.util.ArrayList;
import java.util.Set;

import javax.swing.JFrame;
import javax.swing.UIManager;

import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.motion.RobotMotion;
import edu.illinois.mitra.starlSim.main.SimSettings;


@SuppressWarnings("serial")
public class DrawFrame extends JFrame {
    private DrawPanel dp = null;


    public DrawFrame(Set<String> robotNames, Set<String> blockedWireless, SimSettings settings) {
        try {
            // Set Native Look and Feel
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (Exception e) {
        }

        setTitle("StarL Simulator");
        setSize(1366, 768);
        setLocation(50, 50);

        dp = new DrawPanel(robotNames, blockedWireless, settings);
        dp.setWorld(settings.GRID_XSIZE, settings.GRID_YSIZE);
        dp.setDefaultPosition(-750, -3100, 24);
        getContentPane().add(dp);

    }

    public void updateData(ArrayList<RobotData> data, long time) {
        dp.updateData(data, time);
    }

    public void addPredrawer(Drawer d) {
        dp.addPredrawer(d);
    }

    public void addPointInputAccepter(LogicThread logic) {
        dp.addClickListener(logic);
    }
    public void addKeyInputAccepter(RobotMotion rob) { dp.addKeyListener(rob); }

    public DrawPanel getPanel() {
        return dp;
    }
}
