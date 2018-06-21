import java.io.*;
import java.net.*;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.*;
import lejos.hardware.port.*;
import lejos.hardware.sensor.*;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.*;
import lejos.robotics.localization.*;
import lejos.robotics.navigation.MovePilot;

/**
 * @author malen
 *
 */
public class EV3Control2 {

	private static EV3UltrasonicSensor distSensor;
	private static EV3GyroSensor gyroSensor;
	private static SampleProvider distance;
	private static SampleProvider heading;
	private static float[] distSample;
	private static float[] gyroSample;
	private static EV3LargeRegulatedMotor mL;
	private static EV3LargeRegulatedMotor mR;
	private static EV3MediumRegulatedMotor mB;
	private static MovePilot pilot;
	private static PoseProvider poseProvider;
	private static double radius=300;
	//private static Data data;

	/**
	 * 
	 */
	public EV3Control2() {
		// TODO Auto-generated constructor stub
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) throws IOException{

        String hostName = "MalensPC";
        int portNumber = 444;

        try (
            Socket socket = new Socket(hostName, portNumber);
        	OutputStream outputStream=socket.getOutputStream();
            PrintWriter out = new PrintWriter(outputStream, true);
        	ObjectOutputStream objectOutStream=new ObjectOutputStream(new BufferedOutputStream(outputStream));
            BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            BufferedReader stdIn =new BufferedReader(new InputStreamReader(System.in))
        ) {
        	System.out.println("Connection to PC established!");
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
    		
    		// Motors: 
    		mL=new EV3LargeRegulatedMotor(MotorPort.A); // Left motor
    		mR=new EV3LargeRegulatedMotor(MotorPort.D); // Right motor
    		mB=new EV3MediumRegulatedMotor(MotorPort.B); // Motor controlling platform with the ultrasonic sensor
    		
    		// Chassis and pilot setup
    		Chassis chassis;
            Wheel wheelL = WheeledChassis.modelWheel(mL, 4.2).offset(5.9).invert(true);
            Wheel wheelR = WheeledChassis.modelWheel(mR, 4.2).offset(-5.9).invert(true);
            chassis = new WheeledChassis(new Wheel[]{wheelL, wheelR}, WheeledChassis.TYPE_DIFFERENTIAL);
            pilot = new MovePilot(chassis);
            poseProvider = chassis.getPoseProvider();
            radius = Math.max(radius, pilot.getMinRadius());
            poseProvider = new OdometryPoseProvider(pilot);
            
            pilot.setLinearSpeed(pilot.getMaxLinearSpeed()/2);
            pilot.setLinearAcceleration(pilot.getMaxLinearSpeed()/4);
            pilot.setAngularSpeed(pilot.getMaxAngularSpeed()/2);
            pilot.setAngularAcceleration(pilot.getMaxAngularSpeed()/4);


    		while(true) {
    			double[][] data=scan();
    			objectOutStream.writeObject(data);
//    			heading.fetchSample(gyroSample, 0);
//    			out.println(distSample[0]);
//    			out.println(gyroSample[0]);
//    			out.println("Done scanning");
    			 Button.waitForAnyPress();

    		}
    		

        } catch (UnknownHostException e) {
            System.err.println("Don't know about host " + hostName);
            System.exit(1);
        } catch (IOException e) {
            System.err.println("Couldn't get I/O for the connection to " +
                hostName);
            System.exit(1);
        } 

	}
	
	public static double[][] scan() {
		double[][] dataM = new double[1000][2];
		mB.setSpeed(70);
		mB.rotateTo(360,true);
		int i=0;
		while(mB.isMoving()) {
			distance.fetchSample(distSample, 0);
			dataM[i][0]=mB.getTachoCount();
			dataM[i][1]=distSample[0];
			System.out.println("Angle: "+dataM[i][0]+" Distance reading: "+dataM[i][1]);
			i++;
		}
		System.out.println(i);
		
		mB.rotateTo(0,true);
		while(mB.isMoving()) {
			distance.fetchSample(distSample, 0);
			dataM[i][0]=mB.getTachoCount();
			dataM[i][1]=distSample[0];
			System.out.println("Angle: "+dataM[i][0]+" Distance reading: "+dataM[i][1]);
			i++;
		}
		System.out.println(i);
		
       return dataM;
	}

}
