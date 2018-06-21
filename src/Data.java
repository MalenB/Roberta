
public class Data {

	private transient double[][] dataMatrix;
	private static boolean flag=false;

	public Data() {
		dataMatrix = new double[1000][6];
	}

	public synchronized double[][] getData() {
		while(!flag) {
			try {
				wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		flag = false;
		notifyAll();
		return dataMatrix;
	}

	public void setData(double[][] data) {
		while(flag) {
			try {
				wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		this.dataMatrix = data;
		flag = true;
		notifyAll();

	}

}
