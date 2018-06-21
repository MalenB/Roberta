import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;

public class EV3ComThread extends Thread {

	private static boolean running;
	private Data data;

	public EV3ComThread(Data dataObj) {
		data = dataObj;
	}

	public void run() {
		running = true;
		String hostName = "MalensPC";
		int portNumber = 444;

		try (Socket socket = new Socket(hostName, portNumber);
				OutputStream outputStream = socket.getOutputStream();
				PrintWriter out = new PrintWriter(outputStream, true);
				ObjectOutputStream objectOutStream = new ObjectOutputStream(new BufferedOutputStream(outputStream));
				BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
				BufferedReader stdIn = new BufferedReader(new InputStreamReader(System.in))) {
			System.out.println("Connection to PC established!");

			while (running) {
//				in.read();
				double[][] dataM=data.getData();
				
				// double[][] data=scan();
				objectOutStream.writeObject(dataM);
				// heading.fetchSample(gyroSample, 0);
				// out.println(distSample[0]);
				// out.println(gyroSample[0]);
				// out.println("Done scanning");
//				Button.waitForAnyPress();

			}
			System.err.println("Terminating..");

		} catch (UnknownHostException e) {
			System.err.println("Don't know about host " + hostName);
			System.exit(1);
		} catch (IOException e) {
			System.err.println("Couldn't get I/O for the connection to " + hostName);
			System.exit(1);
		}
	}

	public void terminate() {
		running = false;
	}

}
