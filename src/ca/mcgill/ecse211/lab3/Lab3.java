// Lab3.java
package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.lab3.UltrasonicPoller;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This is the main class that sets up the robot, and starts navigation.
 */
public class Lab3 {
	
  // Route of the robot
  private static final double[][] ROUTE = {{0.0,2.0}, {1.0, 1.0}, {2.0, 2.0}, {2.0, 1.0}, {1.0, 0.0}} ;
  
  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  private static final Port usPort = LocalEV3.get().getPort("S2");
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK = 12.075 ;

  public static void main(String[] args) throws OdometerExceptions {

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // Complete implementation

    Display odometryDisplay = new Display(lcd); // No need to change
    final Navigation nav = new Navigation(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, ROUTE);
    
    @SuppressWarnings("resource") // Because we don't bother to close this resource
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                              // this instance
    float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
                                                         // returned
    
 // Setup Ultrasonic Poller // This thread samples the US and invokes
    UltrasonicPoller usPoller = null; // the selected controller on each cycle
    
    do {
    
      // Start odometer and display threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      
      usPoller = new UltrasonicPoller(usDistance, usData, nav);
      usPoller.start();

      // spawn a new Thread to avoid SquareDriver.drive() from blocking
      (new Thread() {
        public void run() {
          for(int i = 0; i < ROUTE.length; i++) {
        	  nav.travelTo(ROUTE[i][0], ROUTE[i][1], false);
          }
          
        }
      }).start();
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
