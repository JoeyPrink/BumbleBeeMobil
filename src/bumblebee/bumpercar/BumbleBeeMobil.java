package bumblebee.bumpercar;

import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.TouchSensor;
import lejos.nxt.UltrasonicSensor;
//import lejos.robotics.MirrorMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.objectdetection.FeatureDetector;
import lejos.robotics.objectdetection.TouchFeatureDetector;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import lejos.util.PilotProps;

/**
 * 
 TouchSensor(SensorPort.S1);
 UltrasonicSensor(SensorPort.S3);
 */

/**
 * 
 * 
 */
public class BumbleBeeMobil {
	static RegulatedMotor leftMotor;
	static RegulatedMotor rightMotor;
	static TouchSensor touch = new TouchSensor(SensorPort.S1);
	static UltrasonicSensor sonar = new UltrasonicSensor(SensorPort.S3);
	static LightSensor light = new LightSensor(SensorPort.S2);
	static int sonarDistance = 15;
	static int blackWhiteThreshold = 30;
	static DifferentialPilot robot;

	// Use these definitions instead if your motors are inverted
	// static RegulatedMotor leftMotor = MirrorMotor.invertMotor(Motor.A);
	// static RegulatedMotor rightMotor = MirrorMotor.invertMotor(Motor.C);

	public static void main(String[] args) throws Exception {
		PilotProps pp = new PilotProps();
    	pp.loadPersistentValues();
    	float wheelDiameter = Float.parseFloat(pp.getProperty(PilotProps.KEY_WHEELDIAMETER, "4.96"));
    	float trackWidth = Float.parseFloat(pp.getProperty(PilotProps.KEY_TRACKWIDTH, "13.0"));
    	leftMotor = PilotProps.getMotor(pp.getProperty(PilotProps.KEY_LEFTMOTOR, "A"));
    	rightMotor = PilotProps.getMotor(pp.getProperty(PilotProps.KEY_RIGHTMOTOR, "B"));
    	boolean reverse = Boolean.parseBoolean(pp.getProperty(PilotProps.KEY_REVERSE,"false"));
    	
    	robot = new DifferentialPilot(wheelDiameter,trackWidth,leftMotor,rightMotor,reverse);
    	 
        
		boolean started = false;
		System.out
				.println("Hello, my name is BumbleBee! I'm gonna kick ya a**!");
		robot.setAcceleration(4000);
		robot.setTravelSpeed(20); // cm/sec
		robot.setRotateSpeed(180); // deg/sec
		
		Behavior b1 = new DriveForward();
		Behavior b2 = new DetectWall();
		Behavior b3 = new Exit();
		Behavior[] behaviorList = { b1, b2, b3 };
		Arbitrator arbitrator = new Arbitrator(behaviorList);
		// LCD.drawString("Bumper Car", 0, 1);
		Button.waitForAnyPress();
		if (Button.ENTER.isDown()) {
			started = true;
		}
		while (!Button.ESCAPE.isDown() || (!Button.ENTER.isDown() && started)) {
			arbitrator.start();
		}
		stop();

	}

	private static void stop() {
		BumbleBeeMobil.robot.stop();
		System.out.println("Proram stopped!");
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}

class DriveForward implements Behavior {

	private boolean _suppressed = false;

	public boolean takeControl() {
		return true; // this behavior always wants control.
	}

	public void suppress() {
		_suppressed = true;// standard practice for suppress methods
		BumbleBeeMobil.robot.stop();
	}

	public void action() {
		_suppressed = false;
		BumbleBeeMobil.robot.backward();
		while (!_suppressed) {
			Thread.yield(); // don't exit till suppressed
		}
		BumbleBeeMobil.robot.stop();
	}
}

class Exit implements Behavior {

	public Exit() {
	}

	@Override
	public boolean takeControl() {
		return Button.ESCAPE.isPressed();
	}

	@Override
	public void action() {
		BumbleBeeMobil.robot.stop();
		System.exit(0);
	}

	@Override
	public void suppress() {
	}
}

/*class DetectFloorMarks implements Behavior {

	public DetectFloorMarks() {
	}

	@Override
	public boolean takeControl() {
		BumbleBeeMobil.light.readValue();
		if (BumbleBeeMobil.touch.isPressed()) {
			// hit wall in front of me -> turn left
			turn = 1;
		}
		if (BumbleBeeMobil.sonar.getDistance() > BumbleBeeMobil.sonarDistance) {
			// right wall is missing -> turn right
			turn = 2;
		}
		return BumbleBeeMobil.touch.isPressed()
				|| BumbleBeeMobil.sonar.getDistance() > BumbleBeeMobil.sonarDistance;
	}

	@Override
	public void action() {
		BumbleBeeMobil.robot.stop();
		System.exit(0);
	}

	@Override
	public void suppress() {
	}
}*/

class DetectWall implements Behavior {

	// left = 1; right = 2;
	int turn = 0;

	public DetectWall() {
		BumbleBeeMobil.touch = new TouchSensor(SensorPort.S1);
		BumbleBeeMobil.sonar = new UltrasonicSensor(SensorPort.S3);
	}

	public boolean takeControl() {
		BumbleBeeMobil.sonar.ping();
		if (BumbleBeeMobil.touch.isPressed()) {
			// hit wall in front of me -> turn left
			turn = 1;
		}
		if (BumbleBeeMobil.sonar.getDistance() > BumbleBeeMobil.sonarDistance) {
			// right wall is missing -> turn right
			turn = 2;
		}
		return BumbleBeeMobil.touch.isPressed()
				|| BumbleBeeMobil.sonar.getDistance() > BumbleBeeMobil.sonarDistance;
	}

	public void suppress() {
		// Since this is highest priority behavior, suppress will never be
		// called.
	}

	public void action() {

		if (turn == 1) {
			
			// bumper, turn left! 
			Sound.beep();
			BumbleBeeMobil.robot.travel(10);
			BumbleBeeMobil.robot.rotate(-360);
			//BumbleBeeMobil.robot.rotate(720);
			BumbleBeeMobil.robot.travel(-10);
			Sound.beep();

			//TODO LOCK 
			
		} else if (turn == 2) {
			
			// (sensor) missing wall, turn right! 
			BumbleBeeMobil.robot.travel(15);
			
			if (BumbleBeeMobil.sonar.getDistance() < BumbleBeeMobil.sonarDistance)
			{
				turn=0;
				
			}
			else if (BumbleBeeMobil.sonar.getDistance() > BumbleBeeMobil.sonarDistance)
			
			BumbleBeeMobil.robot.rotate(270);
			BumbleBeeMobil.robot.travel(-25);
			
			Sound.twoBeeps();
		}
	}
	

}
