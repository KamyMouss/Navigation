// Lab3.java
package ca.mcgill.ecse211.lab2;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {
	
  // Route of the robot
  private static final double[][] ROUTE = {{1.0,1.0}, {1.0, -1.0}, {-1.0, -1.0}, {-1.0, 1.0}, {0.0, 0.0}} ;
  
  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK = 12.075 ;

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // Complete imlpementation
    OdometryCorrection odometryCorrection = new OdometryCorrection(); // Complete
                                                                      // implementation
    Display odometryDisplay = new Display(lcd); // No need to change
    Navigation nav = new Navigation(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, ROUTE);

    do {
    
      // Start odometer and display threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      
      //Start odometry correction
      //Thread odoCorrectionThread = new Thread(odometryCorrection);
      //odoCorrectionThread.start();
      

      // spawn a new Thread to avoid SquareDriver.drive() from blocking
      (new Thread() {
        public void run() {
          for(int i = 0; i < ROUTE.length; i++) {
        	  Navigation.travelTo(ROUTE[i][0], ROUTE[i][1]);  
          }
          
        }
      }).start();
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
