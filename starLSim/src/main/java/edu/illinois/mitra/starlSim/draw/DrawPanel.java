package edu.illinois.mitra.starlSim.draw;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.event.MouseEvent;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.RoundRectangle2D;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.Set;

import edu.illinois.mitra.starl.interfaces.AcceptsPointInput;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.Obstacles;
import edu.illinois.mitra.starl.objects.Point3d;
import edu.illinois.mitra.starlSim.main.SimSettings;


@SuppressWarnings("serial")
public class DrawPanel extends ZoomablePanel
{
	private ArrayList <RobotData> data = new ArrayList <RobotData>();
	private long time = 0l;
	private long lastUpdateTime = 0l;
	private long startTime = Long.MAX_VALUE;
	private int width = 1, height = 1;
	NumberFormat format = new DecimalFormat("0.00");
	int scaleFactor = 0;
	
	private ArrayList <LinkedList <Point>> robotTraces = new ArrayList <LinkedList <Point>>(); // trace of robot positions
	private final Color TRACE_COLOR = Color.gray;
	
	private LinkedList <Drawer> preDrawers = new LinkedList <Drawer>();
	private LinkedList <AcceptsPointInput> clickListeners = new LinkedList <AcceptsPointInput>();
	
	// wireless interface
	RoundRectangle2D.Double toggle = new RoundRectangle2D.Double(5,5,20,20,15,15);
	boolean showWireless = false;
	ArrayList <String> robotNames = new ArrayList <String>();
	boolean[] wirelessBlocked;
	Set<String> blockedWirelessNames;
	Point clicked = null;

	public DrawPanel(Set<String> robotNames, Set<String> blockedWirelessNames, SimSettings settings)
	{
		super(settings);

		this.robotNames.addAll(robotNames);
		Collections.sort(this.robotNames);
		
		this.blockedWirelessNames = blockedWirelessNames;
		wirelessBlocked = new boolean[robotNames.size()];
	}

	@Override
	protected void draw(Graphics2D g, LogicThread lt)
	{
		Point a = new Point(0, 0);
		Point b = new Point(0, 100);
		g.setStroke(new BasicStroke(10));
		
		synchronized(this)
		{
			for (Drawer d : preDrawers)
				d.draw(lt, g);	
			
			for (int rIndex = 0; rIndex < data.size(); ++rIndex)
			{
				RobotData rd = data.get(rIndex);
				
				
				drawRobot(g,rd,settings.DRAW_ROBOT_TYPE);
				
				if(wirelessBlocked[rIndex]){
					
					drawWorld(g, rd);
				}
				
				// Draw world bounding box
				g.setColor(Color.gray);
				g.setStroke(new BasicStroke(10));
				//g.drawRect(0, 0, width, height);
				
				// Determine scale
				scaleFactor =  (int) toRealCoords(a).distance(toRealCoords(b));
				
				// keep past history of robot positions
				if (settings.DRAW_TRACE) 
				{
					// ensure size
					if (robotTraces.size() != data.size())
					{
						robotTraces.clear();
						
						for (int i = 0; i < data.size(); ++i)
							robotTraces.add(new LinkedList <Point>());
					}
					
					LinkedList <Point> trace = robotTraces.get(rIndex);
					
					if (trace.size() == 0 || trace.getLast().x != rd.x || trace.getLast().y != rd.y)
					{					
						trace.add(new Point(rd.x, rd.y));
						
						if (settings.DRAW_TRACE_LENGTH> 0 && trace.size() > settings.DRAW_TRACE_LENGTH)
							trace.removeFirst();
					}
				}
			}
			
			// draw past history of robot positions
			if (settings.DRAW_TRACE) 
			{
				g.setColor(TRACE_COLOR);
				g.setStroke(new BasicStroke(7));
				for (LinkedList <Point> trace : robotTraces)
				{
					Point last = null;
					
					for (Point p : trace)
					{
						if (last != null)
							g.drawLine(last.x, last.y, p.x, p.y);
						
						last = p;
					}
				}
			}
		}
	}
	
	private void drawWorld(Graphics2D g, RobotData rd)
	{
		g.setStroke(new BasicStroke(10));
		g.setColor(rd.c);
	
		ObstacleList list = rd.world;
		for(int i = 0; i < list.ObList.size(); i++)
		{
			Obstacles currobs = list.ObList.get(i);
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

//		repaint();
	}
	
	private void drawWireless(Graphics2D g)
	{
		int SPACING = 10;
		int SIZE = 15;
		int count = robotNames.size();
		int endY = (int)toggle.y;
		int x = (int)toggle.x;
		int startY = endY - count * (SIZE+SPACING); 
		
		g.setColor(Color.black);
		int curY = startY;
		
		int index = 0;
		for (String s : robotNames)
		{			
			Rectangle r = new Rectangle(x,curY, SIZE, SIZE);
			
			g.draw(r);			
			
			if (!wirelessBlocked[index])
			{
				// draw cross
				g.drawLine(x + 4, curY + 4, x + SIZE - 4, curY + SIZE - 4);
				g.drawLine(x + 4, curY + SIZE - 4, x + SIZE - 4, curY + 4);
			}
			
			
			g.drawString(s + "'s view",x + SIZE + SPACING, curY + SIZE - 3);
			
			curY += (SIZE+SPACING);
			
			++index;
		}
		
	}

	
	@Override
	protected void postDraw(Graphics2D g) {
		g.setColor(Color.black);
		g.setFont(new Font("Tahoma", Font.PLAIN, 15) );
		
		if (startTime == Long.MAX_VALUE) // first time we called postDraw
			startTime = System.currentTimeMillis();
		
		g.drawString((time-startTime)/1000 + " kTic   kTic/Sec:" + format.format(((time-startTime)/1000.0)/((lastUpdateTime-startTime)/1000.0)), 5, getSize().height-5);
		
		g.drawString("SCALE: " + scaleFactor, getSize().width - 125, getSize().height-15);
		g.drawLine(getSize().width - 140, getSize().height-10, getSize().width-40, getSize().height-10);
		
		// toggle
		toggle.y = getHeight() - toggle.height - 30;
		drawToggle(g);
		
		// show the interface to enable / disable wireless for each robot
		if (showWireless){
			drawWireless(g);
		}
	}
	
	public void notifyClickListeners()
	{
		if (clicked != null)
		{
			for (AcceptsPointInput c : clickListeners)
				c.receivedPointInput(clicked.x, clicked.y);
			
			clicked = null;
		}
	}
	
	protected void mousePressedAt(Point p, MouseEvent e) 
	{
		// right click to provide point input
		
		if (e.getButton() == MouseEvent.BUTTON3)
		{
			clicked = p;
		}
	}
	
	public void mousePressed(MouseEvent e) 
	{
		super.mousePressed(e);
		
		Point p = e.getPoint();
		
		if (toggle.contains(p))
		{
			showWireless = !showWireless;
			repaint();
		}
		else if (showWireless)
		{
			// wireless checkboxes
			
			int SPACING = 10;
			int SIZE = 15;
			int count = robotNames.size();
			int endY = (int)toggle.y;
			int x = (int)toggle.x;
			int startY = endY - count * (SIZE+SPACING); 
			int curY = startY;
			
			for (int index = 0; index < robotNames.size(); ++index)
			{			
				Rectangle r = new Rectangle(x,curY, SIZE, SIZE);
				
				if (r.contains(p))
				{
					wirelessBlocked[index] = !wirelessBlocked[index];
					
					if (wirelessBlocked[index])
						blockedWirelessNames.add(robotNames.get(index));
					else
						blockedWirelessNames.remove(robotNames.get(index));
					
					break;
				}
				
				curY += (SIZE+SPACING);
			}
			
		}
	}

	private void drawToggle(Graphics2D g)
	{		
		final int o = 5;
		g.setColor(Color.white);
		g.fill(toggle);
		g.setColor(showWireless ? Color.red : Color.black);
		g.draw(toggle);
		
		g.drawLine((int)toggle.x + o,(int)toggle.y + (int)toggle.height / 2,
				(int)toggle.x + (int)toggle.width - o, (int)toggle.y + (int)toggle.height / 2);
		
		if (!showWireless)
		{ // plus
			g.drawLine((int)toggle.x + (int)toggle.width / 2,(int)toggle.y + o,
					(int)toggle.x + (int)toggle.width / 2, (int)toggle.y + (int)toggle.height - o);
		}
	}
	
	private void drawRobot(Graphics2D g, RobotData rd, boolean drawId){
		g.setStroke(new BasicStroke(this.settings.DRAW_ROBOT_STROKE_SIZE));

		if (rd.c != null)
			g.setColor(rd.c);
		else
			g.setColor(Color.black);
		int radius = settings.BOT_RADIUS;
		if (rd.radius != 0)
			radius = rd.radius;
		
		double radians = 2 * Math.PI * rd.degrees / 360.0;
		
		Point2D.Double from = new Point2D.Double(rd.x, rd.y);
		Point2D.Double to = Geometry.projectPoint(from, radius, radians);
		
		Line2D.Double l = new Line2D.Double(from, to);
		
		g.draw(l);
		if(rd.name.contains(settings.QUADCOPTER_NAME)){
			int outerRadius = (int)Math.round((Math.sqrt(2)-1)*rd.radius);
			int offset = (int) Math.round(Math.sqrt(2)* outerRadius);
			g.drawRect(rd.x-offset/2, rd.y-offset/2, offset, offset);
			//g.drawOval(rd.x - radius, rd.y - radius, radius*2, radius*2);
			g.drawOval(rd.x - outerRadius - outerRadius, rd.y - outerRadius - outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius + outerRadius, rd.y - outerRadius - outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius - outerRadius, rd.y - outerRadius + outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius + outerRadius, rd.y - outerRadius + outerRadius, outerRadius*2, outerRadius*2);
			g.drawString("z: " + rd.z + ", pitch: " + Math.round(rd.pitch*100)/100 + ", roll: " + Math.round(rd.roll*100)/100, rd.x - 55, rd.y + rd.radius + 130);
		}else if(rd.name.contains(settings.GHOST_NAME)) {
			int outerRadius = (int)Math.round((Math.sqrt(2)-1)*rd.radius);
			int offset = (int) Math.round(Math.sqrt(2)* outerRadius);
			g.drawRect(rd.x-offset/2, rd.y-offset/2, offset, offset);
			//g.drawOval(rd.x - radius, rd.y - radius, radius*2, radius*2);
			g.drawOval(rd.x - outerRadius - outerRadius, rd.y - outerRadius - outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius + outerRadius, rd.y - outerRadius - outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius - outerRadius, rd.y - outerRadius + outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius + outerRadius, rd.y - outerRadius + outerRadius, outerRadius*2, outerRadius*2);
			g.drawString("z: " + rd.z + ", pitch: " + Math.round(rd.pitch*100)/100 + ", roll: " + Math.round(rd.roll*100)/100, rd.x - 55, rd.y + rd.radius + 130);
		}
		else if(rd.name.contains(settings.MAVIC_NAME)){
			int outerRadius = (int)Math.round((Math.sqrt(2)-1)*rd.radius);
			int offset = (int) Math.round(Math.sqrt(2)* outerRadius);
			g.drawRect(rd.x-offset/2, rd.y-offset/2, offset, offset);
			//g.drawOval(rd.x - radius, rd.y - radius, radius*2, radius*2);
			g.drawOval(rd.x - outerRadius - outerRadius, rd.y - outerRadius - outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius + outerRadius, rd.y - outerRadius - outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius - outerRadius, rd.y - outerRadius + outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius + outerRadius, rd.y - outerRadius + outerRadius, outerRadius*2, outerRadius*2);
			g.drawString("z: " + rd.z + ", pitch: " + Math.round(rd.pitch*100)/100 + ", roll: " + Math.round(rd.roll*100)/100 + ", yaw: " + rd.yaw, rd.x - 55, rd.y + rd.radius + 130);
		}
		else if(rd.name.contains(settings.PHANTOM_NAME)){
			int outerRadius = (int)Math.round((Math.sqrt(2)-1)*rd.radius);
			int offset = (int) Math.round(Math.sqrt(2)* outerRadius);
			g.drawRect(rd.x-offset/2, rd.y-offset/2, offset, offset);
			//g.drawOval(rd.x - radius, rd.y - radius, radius*2, radius*2);
			g.drawOval(rd.x - outerRadius - outerRadius, rd.y - outerRadius - outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius + outerRadius, rd.y - outerRadius - outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius - outerRadius, rd.y - outerRadius + outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius + outerRadius, rd.y - outerRadius + outerRadius, outerRadius*2, outerRadius*2);
			g.drawString("z: " + rd.z + ", pitch: " + Math.round(rd.pitch*100)/100 + ", roll: " + Math.round(rd.roll*100)/100 + ", yaw: " + rd.yaw, rd.x - 55, rd.y + rd.radius + 130);
		}
		else if(rd.name.contains(settings.o3DR_NAME)){
			int outerRadius = (int)Math.round((Math.sqrt(2)-1)*rd.radius);
			int offset = (int) Math.round(Math.sqrt(2)* outerRadius);
			g.drawRect(rd.x-offset/2, rd.y-offset/2, offset, offset);
			//g.drawOval(rd.x - radius, rd.y - radius, radius*2, radius*2);
			g.drawOval(rd.x - outerRadius - outerRadius, rd.y - outerRadius - outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius + outerRadius, rd.y - outerRadius - outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius - outerRadius, rd.y - outerRadius + outerRadius, outerRadius*2, outerRadius*2);
			g.drawOval(rd.x - outerRadius + outerRadius, rd.y - outerRadius + outerRadius, outerRadius*2, outerRadius*2);
			g.drawString("z: " + rd.z + ", pitch: " + Math.round(rd.pitch*100)/100 + ", roll: " + Math.round(rd.roll*100)/100, rd.x - 55, rd.y + rd.radius + 130);
		}
		else if(rd.name.contains(settings.IROBOT_NAME)){
			g.drawOval(rd.x - radius, rd.y - radius, radius*2, radius*2);
		if(rd.leftbump){
			int x_1 = (int) (rd.radius*(Math.cos(Math.toRadians(rd.degrees+90))) + rd.x);
			int y_1 = (int) (rd.radius*(Math.sin(Math.toRadians(rd.degrees+90))) + rd.y);
			int x_2 = (int) (rd.radius*(Math.cos(Math.toRadians(rd.degrees))) + rd.x);
			int y_2 = (int) (rd.radius*(Math.sin(Math.toRadians(rd.degrees))) + rd.y);
			g.drawLine(x_1, y_1, x_2, y_2);
		}

		
		if(rd.rightbump){
			int x_1 = (int) (rd.radius*(Math.cos(Math.toRadians(rd.degrees))) + rd.x);
			int y_1 = (int) (rd.radius*(Math.sin(Math.toRadians(rd.degrees))) + rd.y);
			int x_2 = (int) (rd.radius*(Math.cos(Math.toRadians(rd.degrees-90))) + rd.x);
			int y_2 = (int) (rd.radius*(Math.sin(Math.toRadians(rd.degrees-90))) + rd.y);
			g.drawLine(x_1, y_1, x_2, y_2);
		}
		}
			
		if (drawId)
		{
			String botType = "default";
			if(rd.type == 0){
				botType =  "goal";
			}
			if(rd.type ==1){
				botType = "discovery";
			}
			if(rd.type ==2){
				botType = "moving obstacle";
			}
			// write name to the right of the robot
			g.drawString(rd.name+" "+ botType , rd.x - 55, rd.y + radius + 50);
		}
		else{
			g.drawString(rd.name+" ", rd.x - 55, rd.y + radius + 50);
		}

	}
	
	public void updateData(ArrayList <RobotData> data, long time)
	{
		synchronized(this)
		{
			this.time = time;
			this.data = data;
			this.lastUpdateTime = System.currentTimeMillis();
		}
		
		repaint();
	}
	
	public void setWorld(int width, int height) {
		synchronized(this)
		{
			this.width = width;
			this.height = height;
		}
	}

	public void addPredrawer(Drawer d)
	{
		synchronized (this)
		{
			preDrawers.add(d);
		}
	}
	
	public void addClickListener(AcceptsPointInput d)
	{
		clickListeners.add(d);
	}
}
