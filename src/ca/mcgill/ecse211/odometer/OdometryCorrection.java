/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private static final double TILE_SIZE = 30.48;
  //colour sensor variables
  private SensorModes myColor;
  private SampleProvider myColorSample;
  private float[] sampleColor;
  private static Port portColor;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

      this.odometer = Odometer.getOdometer();
      //set up colour sensor
	  OdometryCorrection.portColor = LocalEV3.get().getPort("S1");
      this.myColor = new EV3ColorSensor(OdometryCorrection.portColor);
      this.myColorSample = myColor.getMode("Red");
      this.sampleColor = new float[myColor.sampleSize()];
  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  	// run method (required for Thread)
	  public void run() {
	    long correctionStart, correctionEnd;
	    
	    //keep track of which line the robot is at
	    int numLines = 0;
	    //position array for x, y and theta
	    double[] position = new double[3];
	    //vertical or horizontal length
	    boolean vertical = true;
	    //postive or negative (up vs down and right vs left)
	    boolean positive = true;
	    
	
	    while (true) {
	      correctionStart = System.currentTimeMillis();
	      
	      myColorSample.fetchSample(sampleColor, 0);
	      
	      if (sampleColor[0] < 0.35) {
	    	  Sound.beep();
	    	  
	    	  //increase number of lines
	    	  numLines ++;
	    	  
	    	  //get robots current position
	    	  position = odometer.getXYT();
	    	  
	    	  //robot is moving up or down
	    	  if (vertical) {
	    		  //robot is moving up
	    		  if (positive) {
	    			  //correct at each line based on tile size
	    			  if(numLines == 1) {
	    				  odometer.setY(0.0);
	    			  }
	    			  else if(numLines == 2) {
	    				  odometer.setY(TILE_SIZE);
	    			  }
	    			  else if(numLines == 3) {
	    				  odometer.setY(2*TILE_SIZE);
	    				  numLines = 0;
	    				  vertical = false;
	    			  }
	    		  }
	    		  //robot is moving down
	    		  else {
	    			  //correct at each line based on tile size
	    			  if(numLines == 1) {
	    				  odometer.setY(2*TILE_SIZE);
	    			  }
	    			  else if(numLines == 2) {
	    				  odometer.setY(TILE_SIZE);
	    			  }
	    			  else if(numLines == 3) {
	    				  odometer.setY(0.0);
	    				  numLines = 0;
	    				  vertical = false;
	    			  }
	    		  }
	    	  }
	    	  //horizontal
	    	  else {
	    		  //robot is moving right
	    		  if (positive) {
	    			  //correct at each line based on tile size
	    			  if (numLines == 1) {
	    				  odometer.setX(0.0);
	    			  }
	    			  else if (numLines == 2) {
	    				  odometer.setX(TILE_SIZE);
	    			  }
	    			  else if (numLines == 3) {
	    				  odometer.setX(2*TILE_SIZE);
	    				  numLines = 0;
	    				  vertical = true;
	    				  positive = false;
	    			  }
	    		  }
	    		  //robot is moving left
	    		  else {
	    			  //correct at each line based on tile size
	    			  if(numLines == 1) {
	    				  odometer.setX(2*TILE_SIZE);
	    			  }
	    			  else if(numLines == 2) {
	    				  odometer.setX(TILE_SIZE);
	    			  }
	    			  else if(numLines == 3) {
	    				  odometer.setX(0.0);
	    				  numLines = 0;
	    				  vertical = true;
	    			  }
	    		  }
	    		  
	    	  
	    	  }
	    	  
	    	   
	      }
      

	      // this ensure the odometry correction occurs only once every period
	      correctionEnd = System.currentTimeMillis();
	      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
	    	  try {
	    		  Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
	    	  } catch (InterruptedException e) {
	          // there is nothing to be done here
	    	  }
	      }
    }
    
  }

}
