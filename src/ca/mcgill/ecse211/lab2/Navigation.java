package ca.mcgill.ecse211.lab2;

import java.text.DecimalFormat;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
	
	
	private static Odometer odo;
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static double leftRadius;
	private static double rightRadius;
	private static double track;
	
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_SIZE = 30.48;
	
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		      double leftRadius, double rightRadius, double track, double[][] route) throws OdometerExceptions{
		odo = Odometer.getOdometer();
		
		this.leftMotor = leftMotor;
	    this.rightMotor = rightMotor;
	    this.leftRadius = leftRadius;
	    this.rightRadius = rightRadius;
	    this.track = track;
	    
	    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.stop();
		      motor.setAcceleration(3000);
		    }

		    // Sleep for 2 seconds
		    try {
		      Thread.sleep(2000);
		    } catch (InterruptedException e) {
		      // There is nothing to be done here
		    }
	}
	
	//This  method  causes  the  robot  to  travel  to  the  absolute field  location  (x,  y),  specified  in tile points.
	public static void travelTo(double x, double y) {
		/* This  method  should  continuously call turnTo(double theta) and  then set  the motor speed to forward(straight). 
		 *  This will make sure that your heading is updated until you reach your exact goal. 
		 *  This method will poll the odometer for information
		 * */
		double coordinates[] = convertCoordinates(x, y);
		double position[] = odo.getXYT();
		double deltax = coordinates[0] - position[0] ;
		double deltay = coordinates[1] - position[1];
		double angleToRotate = Math.toDegrees(Math.atan(deltax / deltay));
		double distanceToNavigate = Math.sqrt(deltax*deltax + deltay*deltay);
		
		// angle correction algorithm to get range (0-360), tan only give -45 to 45
		if (deltax < 0 && deltay < 0 || deltax >= 0 && deltay < 0) { // in second or third quadrant
			angleToRotate += 180;
		} else if (deltax < 0 && deltay >= 0) { // in fourth quadrant
			angleToRotate += 360;
		} 
		
		turnTo(angleToRotate);
		
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);

        leftMotor.rotate(convertDistance(leftRadius, distanceToNavigate), true);
        rightMotor.rotate(convertDistance(rightRadius, distanceToNavigate), false);
		
//		while (true) {
//			turnTo(theta);
//		     // drive forward three tiles
//		    leftMotor.setSpeed(FORWARD_SPEED);
//		    rightMotor.setSpeed(FORWARD_SPEED);
//
//		    leftMotor.rotate(convertDistance(leftRadius, 3 * TILE_SIZE), true);
//		    rightMotor.rotate(convertDistance(rightRadius, 3 * TILE_SIZE), false);
//		}
	}
	
	public static void turnTo(double theta) {
		/* This method causes the robot to turn (on point) to the absolute heading theta. 
		 * This method should turn a MINIMAL angle to its target.*/
	      // turn 90 degrees clockwise
		
		// Calculate angle to turn to
		int reverse = 1;
		double[] position = odo.getXYT(); 
		double currentAngle = position[2] > 358.0 ? 0.01 : position[2];
		double angleToRotate = (theta > currentAngle) ? theta - currentAngle : currentAngle - theta;
		
		if (angleToRotate > 180) {
			angleToRotate = 360 - angleToRotate;
			reverse = -1;
		}
		
		
	    leftMotor.setSpeed(ROTATE_SPEED);
	    rightMotor.setSpeed(ROTATE_SPEED);
	
	    leftMotor.rotate(reverse * convertAngle(leftRadius, track, angleToRotate), true);
	    rightMotor.rotate(reverse * -convertAngle(rightRadius, track, angleToRotate), false);
	}
	
	public boolean isNavigating() {
		/* This  method  returns  true  if  another  thread  has  called travelTo() or turnTo() 
		 * and  the method has yet to return; false otherwise. */
		return false;
	}
	  
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	}

    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
    
    private static double[] convertCoordinates(double x, double y) {
		double[] coordinates = {x * TILE_SIZE, y * TILE_SIZE};
    	return coordinates;
    	
    }
    
    
}
