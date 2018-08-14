
public class Data {

	private transient double[][] dataMatrix;
	private static boolean flag=false;
	private static boolean pathflag=false;
	private static boolean dataReady=false;
	private static boolean pathReady=false;
	private int numSamples=1000; // Number of samples to record
	private int numVariables=6; // Number of variables to record samples of
	private double[][] path;

	public Data(int numSamples, int numVariables) {
		this.numSamples=numSamples;
		this.numVariables=numVariables;
		dataMatrix = new double[numSamples][numVariables];
		System.out.println("TCreated new data object");
	}

	public synchronized double[][] getData() {
		while(!flag) {
			try {
				wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		System.out.println("Getting data matrix from object");
		flag = false;
		dataReady=false;
		notifyAll();
		return dataMatrix;
	}

	public synchronized void setData(double[][] data) {
		while(flag) {
			try {
				wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		System.out.println("Setting new data matrix in object");
		this.dataMatrix = data;
		dataReady=true;
		flag = true;
		notifyAll();

	}
	
	public synchronized void setDataReady(boolean dataReadyFlag) {
		System.out.println("Setting data ready flag");
		dataReady=dataReadyFlag;
	}
	public synchronized boolean getDataReady() {
//		System.out.println("Checking wether data is ready");
		return dataReady;
	}
	
	public synchronized int getNumSaples() {
		return numSamples;
	}
	public synchronized int getNumVariables() {
		return numVariables;
	}
	
	public synchronized double[][] getPath() {
		while(!pathflag) {
			try {
				wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		System.out.println("Getting path from object");
		pathflag = false;
		pathReady=false;
		notifyAll();
		return path;
	}

	public synchronized void setPath(double[][] newPath) {
		while(pathflag) {
			try {
				wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		System.out.println("Setting new path in object");
		this.path = newPath;
		pathReady=true;
		pathflag = true;
		notifyAll();

	}
	
	public synchronized void setPathReady(boolean pathReadyFlag) {
		System.out.println("Setting path ready flag");
		pathReady=pathReadyFlag;
	}
	public synchronized boolean getPathReady() {
//		System.out.println("Checking wether data is ready");
		return pathReady;
	}


}
