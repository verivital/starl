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
import edu.illinois.mitra.starl.objects.Point3d;
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
			g.fillRect(dest.x() - 13, dest.y() - 13, 26, 26);
			g.drawString(dest.name, dest.x() + 30, dest.y() - 20);
	//		if(!app.destinations.containsKey(dest.name)){
	//			g.drawString(" has been reached", dest.x() + 20, dest.y() - 100);
	//		}
			
		}
/*
		RRTNode curNode = app.kdTree;
		while(curNode != null){
			g.drawRect(curNode.position.x, curNode.position.y, 30, 30);
			if(curNode.parent != null)
			g.drawLine(curNode.position.x, curNode.position.y, curNode.parent.position.x, curNode.parent.position.y);
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
			
			Point3d nextpoint = currobs.obstacle.firstElement();
			Point3d curpoint = currobs.obstacle.firstElement();
			int[] xs = new int[currobs.obstacle.size()]; 
			int[] ys = new int[currobs.obstacle.size()];

			for(int j = 0; j < currobs.obstacle.size() -1 ; j++){
			curpoint = currobs.obstacle.get(j);
			nextpoint = currobs.obstacle.get(j+1);
			g.drawLine(curpoint.x(), curpoint.y(), nextpoint.x(), nextpoint.y());
			xs[j] = curpoint.x();
			ys[j] = curpoint.y();
			}
			xs[currobs.obstacle.size()-1] = nextpoint.x();
			ys[currobs.obstacle.size()-1] = nextpoint.y();
			
			g.drawLine(nextpoint.x(), nextpoint.y(), currobs.obstacle.firstElement().x(), currobs.obstacle.firstElement().y());
			g.fillPolygon(xs,ys,currobs.obstacle.size());
		}
		
		g.setColor(selectColor);
		g.setStroke(stroke);
		if(app.currentDestination != null)
			g.drawOval(app.currentDestination.x() - 20, app.currentDestination.y() - 20, 40, 40);
		g.setColor(Color.BLACK);
		//for(ItemPosition cur: app.doReachavoidCalls){
		//	g.drawRect(cur.x -10, cur.y - 10, 20, 20);
		//	g.drawString(cur.name, cur.x,cur.y);
		//}
	}

}
