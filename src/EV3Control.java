import java.io.*;
import lejos.hardware.motor.*;
import lejos.hardware.port.*;
import lejos.robotics.chassis.*;
import lejos.robotics.localization.*;
import lejos.robotics.navigation.MovePilot;

/**
 * @author malen
 *
 */
public class EV3Control {

	private static EV3LargeRegulatedMotor mL;
	private static EV3LargeRegulatedMotor mR;
	private static EV3MediumRegulatedMotor mB;
	private static MovePilot pilot;
	private static PoseProvider poseProvider;
	private static double radius = 300;
	private static Data data;

	/**
	 * EV3 control
	 */
	public EV3Control() {
		// Motors:
		mL = new EV3LargeRegulatedMotor(MotorPort.A); // Left motor
		mR = new EV3LargeRegulatedMotor(MotorPort.D); // Right motor
		mB = new EV3MediumRegulatedMotor(MotorPort.B); // Motor controlling platform with the ultrasonic sensor

		// Chassis and pilot setup
		Chassis chassis;
		Wheel wheelL = WheeledChassis.modelWheel(mL, 4.2).offset(5.9).invert(true);
		Wheel wheelR = WheeledChassis.modelWheel(mR, 4.2).offset(-5.9).invert(true);
		chassis = new WheeledChassis(new Wheel[] { wheelL, wheelR }, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot = new MovePilot(chassis);
		poseProvider = chassis.getPoseProvider();
		radius = Math.max(radius, pilot.getMinRadius());
		poseProvider = new OdometryPoseProvider(pilot);

		pilot.setLinearSpeed(pilot.getMaxLinearSpeed() / 2);
		pilot.setLinearAcceleration(pilot.getMaxLinearSpeed() / 4);
		pilot.setAngularSpeed(pilot.getMaxAngularSpeed() / 2);
		pilot.setAngularAcceleration(pilot.getMaxAngularSpeed() / 4);
		
		data=new Data();
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) throws IOException {
		new EV3Control();
		Thread com = new EV3ComThread(data);
		com.start();
		Thread scan = new ScanThread(poseProvider, mB, data);
		scan.start();

	}

}
