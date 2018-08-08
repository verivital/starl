package edu.illinois.mitra.demo.lawn;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Stroke;

import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starlSim.draw.Drawer;

public class LawnDrawer extends Drawer {

    private Stroke stroke = new BasicStroke(8);
    private Color selectColor = new Color(0, 0, 255, 100);

    @Override
    public void draw(LogicThread lt, Graphics2D g) {
        LawnMowerApp app = (LawnMowerApp) lt;

        g.setColor(Color.RED);

        //Cosmetics, doesn't change actual waypoints.
        for (ItemPosition dest : app.destinations.values()) {
            g.fillRect(dest.getX() - 13, dest.getY() - 13, 26, 26);
            g.drawString(dest.name, dest.getX() + 30, dest.getY() - 20);
        }

        g.setColor(selectColor);
        g.setStroke(stroke);


        //Draws target ovals. Find way to remove ovals after they're drawn
        if (app.currentDestination != null) {
            g.drawOval(app.currentDestination.getX() - 20, app.currentDestination.getY() - 20, 40, 40);
        }
        g.setColor(Color.GRAY);


        //Code Used to Draw Obstacles


    }
/*
		g.setColor(selectColor);
		g.setStroke(stroke);
		if (app.currentDestination != null)
			g.drawOval(app.currentDestination.getX() - 20, app.currentDestination.getY() - 20, 40, 40);
		g.setColor(Color.BLACK);
		//for(ItemPosition cur: app.doReachavoidCalls){
		//	g.drawRect(cur.getX -10, cur.getY - 10, 20, 20);
		//	g.drawString(cur.name, cur.getX,cur.getY);
		//}
		*/
}

