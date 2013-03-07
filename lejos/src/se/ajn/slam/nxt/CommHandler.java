package se.ajn.slam.nxt;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;
import se.ajn.slam.SLAMCommand;

public class CommHandler implements Runnable {

	public CommHandler(SLAMRobot robot) {
		this.robot = robot;
		this.connected = false;
		this.inputStream = null;
		this.outputStream = null;
	}
	public void run() {
		int ret;
		/* Wait for connection */
		ret = this.connect();
		if (ret != 0)
			return;
		/* Negotiate protocol with other part */
		ret = this.negotiate();
		if (ret != 0)
			return;
		this.signalConnect();
		try {
			int[] cmd;
			while (true) {
				cmd = SLAMCommand.readCommand(this.inputStream);
				if (cmd[0] == SLAMCommand.CMD_ID_HOST_DISCONNECT)
					break;
				ret = this.robot.handleCommand(cmd);
				if (ret < 0)
					break;
			}
		} catch (IOException e) {
			return;
		}
		this.close();
	}
	private int negotiate()
	{
		try {
			int[] cmd = SLAMCommand.readCommand(this.inputStream);
			if (cmd[0] != SLAMCommand.CMD_ID_HOST_CONNECT) {
				return -1;
			}
			int version = cmd[1];
			/* We only support version 1, if version is greater host will have to adapt */
			if (version < 1) {
				return -2;
			}
			
			int[] response = SLAMCommand.createNXTConnect(1);
			this.writeMessage(response);
		} catch (IOException e){
			/* Not much to do here */
			return -3;
		}
		return 0;
	}
	
	public int connect()
	{
		/* Wait for connect from PC */
		this.connection = Bluetooth.waitForConnection();
		this.inputStream = this.connection.openDataInputStream();
		this.outputStream = this.connection.openDataOutputStream();

		return 0;
	}
	
	public void close()
	{
		try {
			this.inputStream.close();
		} catch (IOException e) {}
		
		try {
			this.outputStream.close();
		} catch (IOException e) {}	
	}
	
	public synchronized void waitConnect() {
		while (!this.connected) {
			try {
				wait();
			} catch (InterruptedException e) {
			}
		}
		return;
	}
	
	private synchronized void signalConnect() {
		this.connected = true;
		notifyAll();
	}
	
	public synchronized void writeMessage(int[] msg) throws IOException {
		if (msg.length > SLAMCommand.MAX_LENGTH) {
			throw new IOException("Illegal message size");
		}
		
		for (int i = 0; i < msg.length; i++) {
			this.outputStream.writeInt(msg[i]);
		}
		this.outputStream.flush();
	}
	
	private SLAMRobot robot;
	private BTConnection connection;
	private DataInputStream inputStream;
	private DataOutputStream outputStream;
	private boolean connected; // Access only through synchronized methods!
}
