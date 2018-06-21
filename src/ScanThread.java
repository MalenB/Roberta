import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.PoseProvider;

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
	
	public ScanThread(PoseProvider pp, EV3MediumRegulatedMotor motor, Data dataObj) {
		poseProvider=pp;
		mB=motor;
		data=dataObj;
	}
	
	public void run()
	{
		running=true;
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
		
		while(running) {
			double[][] dataM = new double[1000][6];
			mB.setSpeed(100);
			mB.rotateTo(360,true);
			int i=0;
			while(mB.isMoving()) {
				distance.fetchSample(distSample, 0);
				heading.fetchSample(distSample, 0);
				dataM[i][0]=mB.getTachoCount();
				dataM[i][1]=distSample[0];
				dataM[i][2]=gyroSample[0];
				dataM[i][3]=poseProvider.getPose().getX();
				dataM[i][4]=poseProvider.getPose().getY();
				dataM[i][5]=poseProvider.getPose().getHeading();
				System.out.println("Angle: "+dataM[i][0]+", Distance: "+dataM[i][1]+", Heading: "+dataM[i][2]+
						", X: "+dataM[i][3]+", Y: "+dataM[i][4]+", Heading: "+dataM[i][5]);
				i++;
			}
			System.out.println(i);
			
			mB.rotateTo(0,true);
			while(mB.isMoving()) {
				distance.fetchSample(distSample, 0);
				heading.fetchSample(distSample, 0);
				dataM[i][0]=mB.getTachoCount();
				dataM[i][1]=distSample[0];
				dataM[i][2]=gyroSample[0];
				dataM[i][3]=poseProvider.getPose().getX();
				dataM[i][4]=poseProvider.getPose().getY();
				dataM[i][5]=poseProvider.getPose().getHeading();
				System.out.println("Angle: "+dataM[i][0]+", Distance: "+dataM[i][1]+", Heading: "+dataM[i][2]+
						", X: "+dataM[i][3]+", Y: "+dataM[i][4]+", Heading: "+dataM[i][5]);
				i++;
			}
			System.out.println(i);
			data.setData(dataM);
		}
	
	}
	public void terminate()
	{
		running=false;
	}

}
