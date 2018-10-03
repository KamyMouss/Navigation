package ca.mcgill.ecse211.lab3;

import java.text.DecimalFormat;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to navigate to a given (x,y) location while avoiding obstacles
 */
public class Navigation implements UltrasonicController{
	
	
	private static Odometer odo;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double leftRadius;
	private double rightRadius;
	private double track;
	
	private static final int FORWARD_SPEED = 185;
	private static final int ROTATE_SPEED = 125;
	private static final double TILE_SIZE = 30.48;
	
    private static final int bandCenter = 7; // Offset from the wall (cm)
    private static final int bandwidth = 3; // Width of dead band (cm)
    private static final int motorLow = 50; // Speed of slower rotating wheel (deg/sec)
    private static final int motorHigh = 175; // Speed of the faster rotating wheel (deg/seec)
	private double currentx;
	private double currenty;
	public boolean isNavigating;
	private int pointIndex = 0;

	/**
     * This is the class constructor
	 * 
	 * @param leftMotor, rightMotor, leftRadius, rightRadius, track, route
	 * @throws OdometerExceptions 
	 */
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		      double leftRadius, double rightRadius, double track, double[][] route) throws OdometerExceptions{
		odo = Odometer.getOdometer();
		this.isNavigating = true;
		this.leftMotor = leftMotor;
	    this.rightMotor = rightMotor;
	    this.leftRadius = leftRadius;
	    this.rightRadius = rightRadius;
	    this.track = track;
	    
	    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.stop();
		      motor.setAcceleration(2000);
		    }

		    // Sleep for 2 seconds
		    try {
		      Thread.sleep(2000);
		    } catch (InterruptedException e) {
		      // There is nothing to be done here
		    }
	}
	
	/**
	 * This method moves the robot to a given (x,y) location.
	 * 
	 * @param x, y, fromProcess 
	 */
	public void travelTo(double x, double y, boolean fromProcess) {
		double coordinates[] = convertCoordinates(x, y);
		double position[] = odo.getXYT();
		double deltax = coordinates[0] - position[0];
		double deltay = coordinates[1] - position[1];
		double angleToRotate = Math.toDegrees(Math.atan2(deltax, deltay));
		double distanceToNavigate = Math.hypot(deltax, deltay);

		turnTo(angleToRotate);
		
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);

        leftMotor.rotate(convertDistance(leftRadius, distanceToNavigate), true);
        rightMotor.rotate(convertDistance(rightRadius, distanceToNavigate), false);
        
        this.currentx = x;
		this.currenty = y;
		
		if(fromProcess) {
			return;
		}
        
        while(!isNavigating) {
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
			
			}
		}
    }
	
	/**
	 * This method turns the robot to a given angle.
	 * 
	 * @param theta
	 */
	public void turnTo(double theta) {
		/* This method causes the robot to turn (on point) to the absolute heading theta. 
		 * This method should turn a MINIMAL angle to its target.*/
		
		// Calculate angle to turn to
		double[] position = odo.getXYT(); 
		double angleToRotate = theta - position[2];
		
		if (angleToRotate > 180) {
			angleToRotate = angleToRotate - 360;
		}
		else if (angleToRotate < -180){
			angleToRotate += 360.0;
		}
		
		
	    leftMotor.setSpeed(ROTATE_SPEED);
	    rightMotor.setSpeed(ROTATE_SPEED);
	
	    leftMotor.rotate(convertAngle(leftRadius, track, angleToRotate), true);
	    rightMotor.rotate(-convertAngle(rightRadius, track, angleToRotate), false);
	}
	
	/**
	 * This method converts distance based on wheel radius.
	 * 
	 * @param radius, distance
	 * @return distance
	 */
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * This method converts angle based on wheel radius and width.
	 * 
	 * @param radius, width, angle
	 * @return angle
	 */
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
    
    private static double[] convertCoordinates(double x, double y) {
		double[] coordinates = {x * TILE_SIZE, y * TILE_SIZE};
    	return coordinates;
    	
    }
    
	/**
	 * This method processes the US data.
	 * 
	 * @param distance
	 */
    @Override
    public void processUSData(int distance) {
      
      //if the robot detects a block go around it
      if (distance < 6) {
    	  
    	  //stop the travelTo method
    	  this.isNavigating = false;
    	  leftMotor.setSpeed(ROTATE_SPEED);
    	  rightMotor.setSpeed(ROTATE_SPEED);
    	  leftMotor.rotate(convertAngle(leftRadius, track, 90.0), true);
    	  rightMotor.rotate(-convertAngle(rightRadius, track, 90.0), false);
	    
    	  leftMotor.rotate(convertDistance(leftRadius, 25.0), true);
          rightMotor.rotate(convertDistance(rightRadius, 25.0), false);
          
          leftMotor.rotate(-convertAngle(leftRadius, track, 90.0), true);
    	  rightMotor.rotate(convertAngle(rightRadius, track, 90.0), false);
    	  
    	  leftMotor.rotate(convertDistance(leftRadius, 25.0), true);
          rightMotor.rotate(convertDistance(rightRadius, 25.0), false);
         
          travelTo(currentx, currenty, true);
          //restart the travelTO method for rest of way points
          this.isNavigating = true;
          Thread.currentThread().interrupt();
        	
      }
    }
}
