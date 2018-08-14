import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;

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
		System.out.println("Trying to connect to pc");
		try (Socket socket = new Socket(hostName, portNumber);
				OutputStream outputStream = socket.getOutputStream();
				PrintWriter out = new PrintWriter(outputStream, true);
				ObjectOutputStream objectOutStream = new ObjectOutputStream(new BufferedOutputStream(outputStream));
				
				InputStream inputStream = socket.getInputStream();
				BufferedReader in = new BufferedReader(new InputStreamReader(inputStream));
				BufferedReader stdIn = new BufferedReader(new InputStreamReader(System.in));
				ObjectInputStream inStream = new ObjectInputStream(inputStream); )
		{
			System.out.println("Connection to PC established!");

			while (running) {
//				in.read();
				if(data.getDataReady()) {
					double[][] dataM=data.getData();
					objectOutStream.writeObject(dataM);
					objectOutStream.flush();
					System.out.println("Sent data!");
					//terminate();
				}
				if (inStream.available()>0) {
					double[][] path= (double[][]) inStream.readObject();
					System.out.println("Path received: " + path[0][0] + ", " + path[1][0] + ", " + path[2][0]);
					data.setPath(path);
				}
		

			}
			System.out.println("Terminating..");

		} catch (UnknownHostException e) {
			System.err.println("Don't know about host " + hostName);
			System.out.println(e.getMessage());
			System.exit(1);
		} catch (IOException e) {
			System.err.println("Couldn't get I/O for the connection to " + hostName);
			System.out.println(e.getMessage());
			System.exit(1);
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void terminate() {
		running = false;
	}

}
