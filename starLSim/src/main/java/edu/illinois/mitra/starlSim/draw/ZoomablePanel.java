package edu.illinois.mitra.starlSim.draw;


import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Point;
import java.awt.Polygon;
import java.awt.RenderingHints;
import java.awt.Stroke;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.awt.geom.RoundRectangle2D;
import java.util.Collection;
import java.util.Date;

import javax.swing.JPanel;

import edu.illinois.mitra.starl.interfaces.ExplicitlyDrawable;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starlSim.main.SimSettings;

@SuppressWarnings("serial")
public abstract class ZoomablePanel extends JPanel implements MouseWheelListener, 
MouseListener, MouseMotionListener, ExplicitlyDrawable
{
//	 negative = zoom in, positive = zoom out
	protected double zoomFactor = 0;
	protected double curZoomFactor = 0;
	protected double moveX = 0, moveY = 0;
	private double defaultMoveX = 0, defaultMoveY = 0;
	private int lastX = -1, lastY = -1;
	
	private static final RoundRectangle2D.Double up = new RoundRectangle2D.Double(30,5,20,20,15,15);
	private static final RoundRectangle2D.Double down = new RoundRectangle2D.Double(30,55,20,20,15,15);
	private static final RoundRectangle2D.Double left = new RoundRectangle2D.Double(5,30,20,20,15,15);
	private static final RoundRectangle2D.Double right = new RoundRectangle2D.Double(55,30,20,20,15,15);
	private static final RoundRectangle2D.Double in = new RoundRectangle2D.Double(30,30,10,20,15,15);
	private static final RoundRectangle2D.Double out = new RoundRectangle2D.Double(40,30,10,20,15,15);
	private RoundRectangle2D.Double colored_rect = null;
	
	private static RoundRectangle2D.Double rects[] = 
	{
		up, down, left, right, in, out
	};
	
	private static int[] xUp = {40, 35, 45 };
	private static int[] yUp = {10, 20, 20 };
	
	private static int[] xDown = xUp;
	private static int[] yDown = {70, 60, 60 };
	
	private static int[] xLeft = yUp;
	private static int[] yLeft = xUp;
	
	private static int[] xRight = yDown;
	private static int[] yRight = xUp;
	
	private static Polygon shapes[] = 
	{
		new Polygon( xUp , yUp, 3),
		new Polygon( xDown , yDown, 3),
		new Polygon( xLeft , yLeft, 3),
		new Polygon( xRight , yRight, 3),
		null,null
	};
	
	private static RoundRectangle2D.Double toggle = new RoundRectangle2D.Double(5,5,20,20,15,15);
	
	private boolean moveOn = false;
	private boolean mouseDown = false;
	private Point mousePoint = null;
	protected static Stroke med = new BasicStroke(2);
	private static Stroke thin = new BasicStroke(1);
	
	// this is the image drawn to when drawNow() is called
	private Image drawBuffer = null;
	
	protected SimSettings settings;
	
	public ZoomablePanel(SimSettings settings)
	{
		this.settings = settings;
		addMouseWheelListener(this);
		addMouseListener(this);
		addMouseMotionListener(this);
		
		PaintThread pt = new PaintThread();
		pt.start();
		
		MoveThread mt = new MoveThread();
		mt.start();
	}
	
	protected void preDraw(Graphics2D g)
	{
		
	}
	
	protected void postDraw(Graphics2D g)
	{
		
	}
	
	public abstract void notifyClickListeners();
	
	long lastDrawTime = 0;
	
	public void drawNow(Collection <LogicThread> lts)
	{
		// hackish, also send point input events now (so it's synchronized withl logic threads)
		notifyClickListeners();
		
		long now = System.currentTimeMillis();
		
		final int MIN_REDRAW_MS = 1000 / settings.MAX_FPS;
		
		if (Math.abs(now - lastDrawTime) > MIN_REDRAW_MS) // don't redraw too quickly
		{
			synchronized(this)
			{
				Dimension size = this.getSize();
				
				if(size.getWidth() <= 0 || size.getHeight() <= 0) {
					size.setSize(1, 1);
				}
				
				if (drawBuffer == null || drawBuffer.getWidth(null) != size.width 
						|| drawBuffer.getHeight(null) != size.height)
				{
					// allocate a new image object
					drawBuffer = this.createImage(size.width, size.height);
				}
			
				if (drawBuffer != null)
					forceDrawComponent(drawBuffer.getGraphics(), lts);
			}
			
			lastDrawTime = System.currentTimeMillis(); // use time AFTER the drawing
		}
	}
	
	protected void paintComponent(Graphics g)
	{
		super.paintComponent(g);
		
		synchronized(this)
		{
			if (drawBuffer != null)
				g.drawImage(drawBuffer,0,0,null);
		}
	}
	
	private static Font drawFont = new Font("Tahoma", Font.PLAIN, 100).deriveFont(AffineTransform.getScaleInstance(1, -1));
	
	// draws onto a buffered image which is later placed on the screen
	protected void forceDrawComponent(Graphics g, Collection <LogicThread> lts)
	{
		Graphics2D g2d = (Graphics2D)g;
		
		setupDrawing(g);
		
		AffineTransform a = g2d.getTransform();
		myPreDraw(g2d);
		g2d.setColor(Color.black);
		
//		Font f = new Font("Tahoma", Font.PLAIN, 55); // TODO: make this configurable in SimSettings
//		f = f.deriveFont(AffineTransform.getScaleInstance(1, -1)); // flip getY back around
		g2d.setFont(drawFont);
		
		AffineTransform preDrawersTransform = g2d.getTransform();
		
		for (LogicThread lt : lts)
		{
			g2d.setTransform(preDrawersTransform);
			g2d.setColor(Color.black);
			g2d.setStroke(med);
			
			draw(g2d, lt);
			
		
//			if (g2d.getFont() != f)
//				throw new RuntimeException("Font was changed in draw method. You should use Font.deriveFont instead and then restore it before your method returns.");
		}
		
		g2d.setTransform(a);
		g2d.setStroke(med);
		
		if (moveOn)
		{
			for (int x = 0; x < rects.length; ++x)
			{
				g.setColor(Color.white);
				g2d.fill(rects[x]);
				
				if (rects[x] == colored_rect)
					g.setColor(Color.red);				
				else
					g.setColor(Color.black);
				
				g2d.draw(rects[x]);
				
				if (x > 3)
				{	
					Point middle = new Point(40,40);
					final int o = 2;
					g2d.setStroke(thin);
					if (x == 4)
					{
						middle = new Point(middle.x-5,middle.y);
						Point top = new Point(middle.x,middle.y-o);
						Point bottom = new Point(middle.x,middle.y+o);
						Point left = new Point(middle.x-o,middle.y);
						Point right = new Point(middle.x+o,middle.y);
						
						g2d.drawLine(top.x,top.y,bottom.x,bottom.y);
						g2d.drawLine(left.x,left.y,right.x,right.y);
					}
					else if (x == 5)
					{
						middle = new Point(middle.x+5,middle.y);
						
						Point left = new Point(middle.x-o,middle.y);
						Point right = new Point(middle.x+o,middle.y);
						
						g2d.drawLine(left.x,left.y,right.x,right.y);
					}
					
					g2d.setStroke(med);
				}
				else if (shapes[x] != null)
				{
						g2d.fill(shapes[x]);
				}
			}
		}
		
		drawToggle(g2d);
		
		postDraw(g2d);
	}

	/*
	 * Set the default position for this panel and move to it
	 */
	public void setDefaultPosition(double x, double y, double z)
	{
		moveX = defaultMoveX = -x;
		moveY = defaultMoveY = -y;
		
		zoomFactor = curZoomFactor = z;
	}
	
	/*
	 * Set the default position for this panel and move to it
	 */
	public Point2D.Double getDefaultPosition()
	{
		return new Point2D.Double(-defaultMoveX, -defaultMoveY);
	}
	
	/*
	 * Set the position and repaint
	 */
	public void setPosition(double x, double y)
	{
		moveX = -x;
		moveY = -y;
		
		repaint();
	}
	
	/**
	 * Move to the default position
	 */
	public void defaultPosition() 
	{
		moveX = defaultMoveX;
		moveY = defaultMoveY;
			
		zoomFactor = curZoomFactor = 0;
		repaint();
	}
	
	/**
	 * @param g the graphics object to use
	 */
	private void drawToggle(Graphics2D g)
	{
		final int o = 5;
		g.setColor(Color.white);
		g.fill(toggle);
		g.setColor(moveOn ? Color.red : Color.black);
		g.draw(toggle);
		
		g.drawLine((int)toggle.x + o,(int)toggle.y + (int)toggle.height / 2,
				(int)toggle.x + (int)toggle.width - o, (int)toggle.y + (int)toggle.height / 2);
		
		if (!moveOn)
		{ // plus
			g.drawLine((int)toggle.x + (int)toggle.width / 2,(int)toggle.y + o,
					(int)toggle.x + (int)toggle.width / 2, (int)toggle.y + (int)toggle.height - o);
		}
	}
	
	private void setupDrawing(Graphics g)
	{
		Graphics2D g2 = (Graphics2D)g;
		// Enable Anti-Aliasing
		g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);   
		g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
	}
	
	protected abstract void draw(Graphics2D g, LogicThread lt);
	
	private double getScale()
	{
		double scale = Math.exp(-curZoomFactor/15);
		
		return scale;
	}
	
	private void myPreDraw(Graphics2D g)
	{
		Dimension size = getSize();
		g.setColor(Color.white);
		g.fillRect(0,0,size.width,size.height);
		
		preDraw(g);
		
		double scale = getScale();
		
		g.scale(scale,scale);
		
		// move
		g.translate(moveX,moveY);

		// flip getY axis
		g.scale(1,-1);
	}

	public void mouseWheelMoved(MouseWheelEvent e)
	{
		//System.out.println("Mouse Wheel moved");
		
		int num = e.getWheelRotation();
		
		zoomFactor += num;
		
		repaint();
	}

	public void mouseClicked(MouseEvent arg0){	}
	public void mousePressed(MouseEvent e) 
	{  
		boolean handled = false;
		
		
		/*if (e.getClickCount() == 2)
		{
			defaultPosition();				
		}*/
		
		Point p = e.getPoint();
		
		lastX = p.x;
		lastY = p.y;
		
		
		if (e.getButton() == MouseEvent.BUTTON1)
		{
			if (toggle.contains(p))
			{
				moveOn = !moveOn;
				repaint();
				handled = true;
			}
			else if (moveOn)
			{								
				for (int x = 0; x < rects.length; ++x)
				{
					if (rects[x].contains(p))
					{
						colored_rect = rects[x];
						
						// set mouseDown
						mouseDown = true;
						mousePoint = p;
						handled = true;
					}
				}
			}
		}
		
		if (!handled)
		{
			mousePressedAt(toRealCoords(e.getPoint()), e);
		}
	}

	/**
	 * Is pressing the mouse button here a zoom action? 
	 * @param p the point where the mouse was pressed 
	 * @return true iff it was a zoom/scroll action
	 */
	public boolean isZoomAction(Point p)
	{
		boolean rv = false;
		
		if (toggle.contains(p))
		{
			rv = true;
		}
		else if (moveOn)
		{
			if (up.contains(p)) rv = true;
			else if (down.contains(p)) rv = true;
			else if (left.contains(p)) rv = true;
			else if (right.contains(p)) rv = true;
			else if (in.contains(p)) rv = true;
			else if (out.contains(p)) rv = true;
		}
		
		return rv;
	}
	
	public Point toRealCoords(Point p)
	{
		double scale = getScale();
		
		return new Point((int)(p.x/scale - moveX) ,(int)-(p.y/scale - moveY) );
	}
	
	public void mouseReleased(MouseEvent e) 
	{  
		if (e.getButton() == MouseEvent.BUTTON2)
		{
			Point p = e.getPoint();
			
			double dx = lastX - p.x;
			double dy = p.y - lastY;
			double scale = getScale();
			
			moveX += dx/scale;
			moveY += dy/scale;
			
			lastX = lastY = -1;
			
			repaint();
		}
		if (e.getButton() == MouseEvent.BUTTON1)
		{
			if (colored_rect != null)
			{
				colored_rect = null;
				repaint();
			}
			
			mouseDown = false;
		}
		
		mouseReleasedAt(toRealCoords(e.getPoint()), e);
	}
	
	public void mouseEntered(MouseEvent e) 
	{  
		mouseEnteredAt(toRealCoords(e.getPoint()), e);
	}
	
	public void mouseExited(MouseEvent e) 
	{ 
		mouseDown = false; 
		
		mouseExitedAt(toRealCoords(e.getPoint()), e);
	}
	
	protected void mousePressedAt(Point p, MouseEvent e) {}
	protected void mouseDraggedAt(Point realPoint, MouseEvent e) {}
	protected void mouseReleasedAt(Point realPoint, MouseEvent e) {}
	protected void mouseEnteredAt(Point realPoint, MouseEvent e) {}
	protected void mouseExitedAt(Point realPoint, MouseEvent e) {}
	protected void mouseMovedAt(Point realPoint, MouseEvent e) {}
	
	public void mouseDragged(MouseEvent e) 
	{  		 
		if (lastX != -1 && lastY != -1)
		{
			Point p = e.getPoint();
			
			double dx = p.x - lastX;
			double dy = p.y - lastY;			
			double scale = getScale();
			
			moveX += dx/scale;
			moveY += dy/scale;
			
			lastX = p.x;
			lastY = p.y;
			
			repaint();
		}
		
		mouseDraggedAt(toRealCoords(e.getPoint()), e);
	}
	
	public void mouseMoved(MouseEvent e) 
	{  
		mouseMovedAt(toRealCoords(e.getPoint()), e);
	}
	
	class MoveThread extends Thread
	{
		public void run()
		{
			long lastTime = -1;
			
			while (true)
			{	
				if (mouseDown)
				{
					long time  = new Date().getTime();
					double scale = getScale();
					double w = getWidth() / scale;
					double h = getHeight() / scale;
					long dif = time - lastTime;
					
					if (lastTime == -1)
					{
						if (out.contains(mousePoint))
							curZoomFactor += 0.1;
						else if (in.contains(mousePoint))
							curZoomFactor -= 0.1;
						else if (up.contains(mousePoint))
							moveY += 5;
						else if (down.contains(mousePoint))
							moveY -= 5;
						else if (left.contains(mousePoint))
							moveX += 5;
						else if (right.contains(mousePoint))
							moveX -= 5;
					}
					else
					{
						double change = dif / 100.0;
						
						if (out.contains(mousePoint))
							curZoomFactor += change;
						else if (in.contains(mousePoint))
							curZoomFactor -= change;
						else if (up.contains(mousePoint))
							moveY += (int)(change * 50);
						else if (down.contains(mousePoint))
							moveY -= (int)(change * 50);
						else if (left.contains(mousePoint))
							moveX += (int)(change * 50);
						else if (right.contains(mousePoint))
							moveX -= (int)(change * 50);
					}					
					
					double scaleAfter = getScale();
					double wAfter = getWidth() / scaleAfter;
					double hAfter = getHeight() / scaleAfter;		
					
					double xGained = (wAfter - w);
					double yGained = (hAfter - h);
					
					double dx = xGained/2;
					double dy = yGained/2;
					
					moveX += dx;
					moveY += dy;					
					
					zoomFactor = curZoomFactor;
					repaint();
					lastTime = time;
				}
				else
					lastTime = -1;
				
				try
				{
					Thread.sleep(10);
				}
				catch (Exception e) { }
			}
		}
	}
	
	class PaintThread extends Thread 
	{
		public void run() 
		{
			long lastTime = -1;
			
			while (true)
			{	
				if (Math.abs(curZoomFactor-zoomFactor) > 0.5)
				{
					long time  = new Date().getTime();
					double scale = getScale();
					double w = getWidth() / scale;
					double h = getHeight() / scale;
					long dif = time - lastTime;
					
					if (lastTime == -1)
					{
						if (curZoomFactor < zoomFactor)
							curZoomFactor += 0.1;
						else 
							curZoomFactor -= 0.1;
					}
					else
					{
						double change = dif / 100.0;
						double error = Math.abs(zoomFactor -curZoomFactor);
						double speedup = 1.0;
						
						if (error > 5)
							speedup = (error-3.0)/2.0;			
						
						if (curZoomFactor < zoomFactor)
							curZoomFactor += change * speedup;
						else 
							curZoomFactor -= change * speedup;
					}
					
					
					double scaleAfter = getScale();
					double wAfter = getWidth() / scaleAfter;
					double hAfter = getHeight() / scaleAfter;		
					
					double xGained = (wAfter - w);
					double yGained = (hAfter - h);
					
					double dx = xGained/2;
					double dy = yGained/2;
					
					moveX += dx;
					moveY += dy;					
					
					repaint();
					lastTime = time;
				}
				else
					lastTime = -1;
				
				try
				{
					Thread.sleep(10);
				}
				catch (Exception e) { }
			}
		}
	}
}
