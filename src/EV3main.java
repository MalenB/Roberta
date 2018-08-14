import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;

public class EV3main extends Thread {
	
	private static EV3LargeRegulatedMotor mL;
	private static EV3LargeRegulatedMotor mR;
	private static EV3MediumRegulatedMotor mB;
	private static MovePilot pilot;
	private static PoseProvider poseProvider;
	private static double radius = 300;
	private static Data data;
	private static int numSamples=200; // Number of samples to record
	private static int numVariables=2; // Number of variables to record samples of
	private EV3GyroSensor gyroSensor;
	private static SampleProvider heading;
	private static float[] gyroSample;
	private EV3UltrasonicSensor distSensor;
	private static Navigator navigator;
	private static SampleProvider distance;
	private static float[] distSample;

	public EV3main() {
		// Sensors:
    	// Get ports for sensors
		lejos.hardware.port.Port distPort = LocalEV3.get().getPort("S1");
		lejos.hardware.port.Port gyroPort = LocalEV3.get().getPort("S4");
		// Create new sensor objects
		distSensor = new EV3UltrasonicSensor(distPort);
		gyroSensor = new EV3GyroSensor(gyroPort);
		// Get a sample provider for these sensors in the specified measurement modes
		distance= distSensor.getDistanceMode();
		heading= gyroSensor.getAngleMode();
		// Create arrays for fetching samples (size specified by the sample provider).
		distSample = new float[distance.sampleSize()];
		gyroSample = new float[heading.sampleSize()];
		
		mL = new EV3LargeRegulatedMotor(MotorPort.A); // Left motor
		mR = new EV3LargeRegulatedMotor(MotorPort.D); // Right motor
		mB = new EV3MediumRegulatedMotor(MotorPort.B); // Motor controlling platform with the ultrasonic sensor

		// Chassis and pilot setup
		Chassis chassis;
		Wheel wheelL = WheeledChassis.modelWheel(mL, 0.042).offset(0.059).invert(true);
		Wheel wheelR = WheeledChassis.modelWheel(mR, 0.042).offset(-0.059).invert(true);
		chassis = new WheeledChassis(new Wheel[] { wheelL, wheelR }, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot = new MovePilot(chassis);
		poseProvider = chassis.getPoseProvider();
		radius = Math.max(radius, pilot.getMinRadius());
		poseProvider = new OdometryPoseProvider(pilot);

		pilot.setLinearSpeed(pilot.getMaxLinearSpeed() / 4);
		pilot.setLinearAcceleration(pilot.getMaxLinearSpeed() / 6);
		pilot.setAngularSpeed(pilot.getMaxAngularSpeed() / 4);
		pilot.setAngularAcceleration(pilot.getMaxAngularSpeed() / 6);
		
		navigator=new Navigator(pilot, poseProvider);
		
		data=new Data(numSamples, numVariables);
	}

	public static void main(String[] args) {
		new EV3main();
		Thread com = new EV3ComThread(data);
		com.start();
		scan();
		
		while(true) {
			
			if(data.getPathReady()) {
				followPath(data.getPath());
			}
		}

	}
	
	public static void scan()
	{
	
		double[][] dataM = new double[data.getNumSaples()][data.getNumVariables()];
		heading.fetchSample(gyroSample, 0);
		dataM[0][0]=poseProvider.getPose().getX();
		dataM[0][1]=poseProvider.getPose().getY();
		dataM[1][0]=gyroSample[0];
		dataM[1][1]=poseProvider.getPose().getHeading();
		System.out.println("X: "+dataM[0][0]+", Y: "+dataM[0][1]);
		System.out.println("Angle: "+dataM[1][0]+", Distance: "+dataM[1][1]);
		int i=2;
		if (Math.abs(mB.getTachoCount()-360)<5)
		{
			mB.rotateTo(0,true);
		}else if (Math.abs(mB.getTachoCount())<5)
		{
			mB.rotateTo(360,true);
		}
		
		while(mB.isMoving()) {
			distance.fetchSample(distSample, 0);
			dataM[i][0]=mB.getTachoCount();
			dataM[i][1]=distSample[0];
			System.out.println("Angle: "+dataM[i][0]+", Distance: "+dataM[i][1]);
			i++;
		}
		System.out.println(i);
		
	
		data.setData(dataM);

	}
	
	public static void followPath(double[][] points) {
		for (int i=0; i<=points.length;i++){
			navigator.goTo((float)points[i][0], (float)points[i][1], (float)points[i][2]);
			while(navigator.isMoving()) {};
			scan();
		}
	}

}
