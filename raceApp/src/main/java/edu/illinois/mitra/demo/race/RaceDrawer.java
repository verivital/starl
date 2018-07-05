package edu.illinois.mitra.demo.race;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Stroke;

import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.Obstacles;
import edu.illinois.mitra.starl.objects.Point3i;
import edu.illinois.mitra.starlSim.draw.Drawer;

public class RaceDrawer extends Drawer {

	private Stroke stroke = new BasicStroke(8);
	private Color selectColor = new Color(0,0,255,100);
	
	@Override
	public void draw(LogicThread lt, Graphics2D g) {
		RaceApp app = (RaceApp) lt;

		g.setColor(Color.RED);
		
		int i = 0;
		for(ItemPosition dest : app.destinationsHistory) {
			i++;
			g.fillRect(dest.getX() - 13, dest.getY() - 13, 26, 26);
			g.drawString(dest.name, dest.getX() + 30, dest.getY() - 20);
	//		if(!app.destinations.containsKey(dest.name)){
	//			g.drawString(" has been reached", dest.getX() + 20, dest.getY() - 100);
	//		}
			
		}
/*
		RRTNode curNode = app.kdTree;
		while(curNode != null){
			g.drawRect(curNode.position.getX, curNode.position.getY, 30, 30);
			if(curNode.parent != null)
			g.drawLine(curNode.position.getX, curNode.position.getY, curNode.parent.position.getX, curNode.parent.position.getY);
			curNode = curNode.parent;
		}
		*/
		g.setColor(Color.GRAY);
		ObstacleList list = app.obs;
		for(int k = 0; k < list.ObList.size(); k++)
		{
			Obstacles currobs = list.ObList.get(k);
			if(currobs.hidden)
				g.setColor(Color.LIGHT_GRAY);
			else
				g.setColor(Color.GRAY);
			
			Point3i nextpoint = currobs.obstacle.firstElement();
			Point3i curpoint = currobs.obstacle.firstElement();
			int[] xs = new int[currobs.obstacle.size()]; 
			int[] ys = new int[currobs.obstacle.size()];

			for(int j = 0; j < currobs.obstacle.size() -1 ; j++){
			curpoint = currobs.obstacle.get(j);
			nextpoint = currobs.obstacle.get(j+1);
			g.drawLine(curpoint.getX(), curpoint.getY(), nextpoint.getX(), nextpoint.getY());
			xs[j] = curpoint.getX();
			ys[j] = curpoint.getY();
			}
			xs[currobs.obstacle.size()-1] = nextpoint.getX();
			ys[currobs.obstacle.size()-1] = nextpoint.getY();
			
			g.drawLine(nextpoint.getX(), nextpoint.getY(), currobs.obstacle.firstElement().getX(), currobs.obstacle.firstElement().getY());
			g.fillPolygon(xs,ys,currobs.obstacle.size());
		}
		
		g.setColor(selectColor);
		g.setStroke(stroke);
		if(app.currentDestination != null)
			g.drawOval(app.currentDestination.getX() - 20, app.currentDestination.getY() - 20, 40, 40);
		g.setColor(Color.BLACK);
		//for(ItemPosition cur: app.doReachavoidCalls){
		//	g.drawRect(cur.getX -10, cur.getY - 10, 20, 20);
		//	g.drawString(cur.name, cur.getX,cur.getY);
		//}
	}

}
