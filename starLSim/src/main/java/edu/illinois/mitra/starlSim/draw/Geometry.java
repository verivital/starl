package edu.illinois.mitra.starlSim.draw;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;


public class Geometry
{
	public static double SMALL = 0.0000001;

	public static double getLength(Line2D.Double line)
	{
		double dx = line.x1 - line.x2;
		double dy = line.y1 - line.y2;
		
		return (float)Math.sqrt(dx*dx + dy*dy);
	}

	/**
	 * up is positive infinity, down is negative infinity, NAN is if start == end
	 * @param line
	 * @return
	 */
	public static double getSlope(Line2D.Double line)
	{
		double rv = Double.NaN;
		double dx = line.x2 - line.x1;
		double dy = line.y2 - line.y1;
		
		if (Math.abs(dx) < SMALL)
		{
			// dx is zero
			
			if (dy > SMALL)
				rv = Double.POSITIVE_INFINITY;
			else if (dy < -SMALL)
				rv = Double.NEGATIVE_INFINITY;
		}
		else
		{
			// dx is nonzero
			rv = dy / dx;
		}
		
		return rv;
	}

	public static double getAngle(Line2D.Double line)
	{
		double dx = line.x2 - line.x1;
		double dy = line.y2 - line.y1;
		
		return Math.atan2(dy, dx);
	}

	public static Point2D.Double projectPoint(Point2D.Double orgin, double magnitude, double radians)
	{
		double x = magnitude * Math.cos(radians);
		double y = magnitude * Math.sin(radians);
		
		return new Point2D.Double(orgin.x + x, orgin.y + y);
	}
	
	public static double segSegDist(Line2D.Double one, Line2D.Double two)
	{
		double rv = Double.MAX_VALUE;
		
		if (one.intersectsLine(two) && getLength(one) > 0 && getLength(two) > 0)
		{
			rv = 0;
		}
		else
		{
			rv = Math.min(rv, one.ptSegDist(two.getP1()));
			rv = Math.min(rv, one.ptSegDist(two.getP2()));
			
			rv = Math.min(rv, two.ptSegDist(one.getP1()));
			rv = Math.min(rv, two.ptSegDist(one.getP2()));
		}
		
		return rv;
	}
	
	public static Point2D.Double segSegIntersection(Line2D.Double one, Line2D.Double two)
	{
		Point2D.Double rv = null;
		
		if (getLength(one) < SMALL) // line one is actually a point
		{
			if (two.ptSegDist(one.getP1()) < SMALL)
				rv = new Point2D.Double(one.x1, one.y1);
		}
		else if (getLength(two) < SMALL) // line two is actually a point
		{
			if (one.ptSegDist(two.getP1()) < SMALL)
				rv = new Point2D.Double(two.x1, two.y1);
		}
		else if (one.intersectsLine(two))
		{
			double m1 = getSlope(one);
			double m2 = getSlope(two);
			
			if (m1 == Double.POSITIVE_INFINITY || m1 == Double.NEGATIVE_INFINITY)
			{
				// one is a vertical line, use its getX value and solve for getY
				double b2 = getYIntercept(two);
				
				double x = one.x1;
				double y = m2 * x + b2;
				
				rv = new Point2D.Double(x, y);
			}
			else if (m2 == Double.POSITIVE_INFINITY || m2 == Double.NEGATIVE_INFINITY)
			{
				// two is a vertical line, use its getX value and solve for getY
				double b1 = getYIntercept(one);
				
				double x = two.x1;
				double y = m1 * x + b1;
				
				rv = new Point2D.Double(x, y);
			}
			else if (Math.abs(m1 - m2) < SMALL)
			{
				// lines are parallel, intersection must be at an end point
				
				Point2D.Double a = new Point2D.Double(one.x1, one.y1);
				Point2D.Double b = new Point2D.Double(one.x2, one.y2);
				
				if (two.ptSegDist(a) < two.ptSegDist(b))
					rv = a;
				else
					rv = b;
			}
			else
			{
				// neither is a vertical line and lines are not parallel
				double b1 = getYIntercept(one);
				double b2 = getYIntercept(two);
				
				double x = (b2 - b1) / (m1 - m2);
				double y = m1 * x + b1;
				
				rv = new Point2D.Double(x, y);
			}				
		}
		
		return rv;
	}

	private static double getYIntercept(Line2D.Double line)
	{
		double m = getSlope(line);
		double dx = line.x1;
		
		return line.y1 - m * dx;
	}
	
}
