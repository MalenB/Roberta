import lejos.hardware.Button;
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
import lejos.robotics.navigation.Pose;

public class ScanThread extends Thread {

	private PoseProvider poseProvider;
	private boolean running;
	private EV3MediumRegulatedMotor mB;
	private EV3UltrasonicSensor distSensor;
	private EV3GyroSensor gyroSensor;
	private SampleProvider distance;
	private SampleProvider heading;
	private float[] distSample;
	private float[] gyroSample;
	private Data data;
	private EV3LargeRegulatedMotor mL;
	private EV3LargeRegulatedMotor mR;
	private static MovePilot pilot;
	private double radius=300;
	
	private double[][] dataM; // for testing
	
	public ScanThread(Data dataObj) {
		data=dataObj;
		running=true;
		// Sensors:
    	// Get ports for sensors
		//lejos.hardware.port.Port distPort = LocalEV3.get().getPort("S1");
		lejos.hardware.port.Port gyroPort = LocalEV3.get().getPort("S4");
		// Create new sensor objects
		//distSensor = new EV3UltrasonicSensor(distPort);
		gyroSensor = new EV3GyroSensor(gyroPort);
		// Get a sample provider for these sensors in the specified measurement modes
		//distance= distSensor.getDistanceMode();
		heading= gyroSensor.getAngleMode();
		// Create arrays for fetching samples (size specified by the sample provider).
		//distSample = new float[distance.sampleSize()];
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

//		pilot.setLinearSpeed(pilot.getMaxLinearSpeed() / 2);
//		pilot.setLinearAcceleration(pilot.getMaxLinearSpeed() / 4);
//		pilot.setAngularSpeed(pilot.getMaxAngularSpeed() / 2);
//		pilot.setAngularAcceleration(pilot.getMaxAngularSpeed() / 4);
		pilot.setLinearSpeed(pilot.getMaxLinearSpeed() / 4);
		pilot.setLinearAcceleration(pilot.getMaxLinearSpeed() / 6);
		pilot.setAngularSpeed(pilot.getMaxAngularSpeed() / 4);
		pilot.setAngularAcceleration(pilot.getMaxAngularSpeed() / 6);
		
		dataM = new double[data.getNumSaples()][data.getNumVariables()]; // for testing
		
	}
	
	public void run()
	{
//		double[][] dataM = new double[data.getNumSaples()][data.getNumVariables()];
//		int i=0;
		mB.setSpeed(70);
		double dist=0;
		int i=0;
		Pose startPose=poseProvider.getPose();
		while(running) {
			//scan();
			//dist=dist+0.1;
			move(-360);
			dataM[i][0]=poseProvider.getPose().getX();
			dataM[i][1]=poseProvider.getPose().getY();
			dataM[i+1][0]=gyroSample[0];
			dataM[i+1][1]=poseProvider.getPose().getHeading();
			System.out.println(dataM[i][0]+", "+dataM[i][1]+", "+dataM[i+1][0]+", "+dataM[i+1][1]+", ");
			 Button.waitForAnyPress();
			 poseProvider.setPose(startPose);		 
			
			if (i>5)
			{
				data.setData(dataM);
				terminate();
			}
			i++;
			
		}
		System.out.println("stopped thread");
	
	}
	public void terminate()
	{
		running=false;
	}

	public void scan()
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
		mB.rotateTo(360,true);
		while(mB.isMoving()) {
			distance.fetchSample(distSample, 0);
			dataM[i][0]=mB.getTachoCount();
			dataM[i][1]=distSample[0];
			System.out.println("Angle: "+dataM[i][0]+", Distance: "+dataM[i][1]);
			i++;
		}
		System.out.println(i);
		
		mB.rotateTo(0,true);
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
	public static void move(double angle) {
		
		//pilot.travel(dist);
		//pilot.arc(0, -90);
		pilot.rotate(angle);


	}

}
