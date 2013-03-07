package se.ajn.slam.pchost;

import lejos.pc.comm.*;
import java.io.*;
import java.util.concurrent.*;

import se.ajn.slam.*;

/**
 * You need pccomm.jar and bluecove.jar on the CLASSPATH. 
 * On Linux, you will also need bluecove-gpl.jar on the CLASSPATH.
 *
 */
public class PCBridge {	
		
	public static void main(String[] args) {
		NXTConnector conn = new NXTConnector();

		conn.addLogListener(new NXTCommLogListener(){

			public void logEvent(String message) {
				System.out.println("BTSend Log.listener: "+message);

			}

			public void logEvent(Throwable throwable) {
				System.out.println("BTSend Log.listener - stack trace: ");
				throwable.printStackTrace();
			}

		});
		// Connect to any NXT over Bluetooth
		boolean connected = conn.connectTo("btspp://");


		if (!connected) {
			System.err.println("Failed to connect to any NXT");
			System.exit(1);
		}

		PCBridge pcb = new PCBridge(conn);
		
		/* Start support threads */
		pcb.start();
		/* Wait for and handle incoming commands */
		pcb.processCommands();
		/* Close spawned threads */
		pcb.stop();
		
		try {
			conn.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		/* Force quit, killing any remaining subthreads */
		System.exit(0);
	}
	
	public PCBridge(NXTConnector conn) {
		this.dos = conn.getDataOut();
		this.dis = conn.getDataIn();
		this.pcin = new DataInputStream(System.in);
		this.queue = new LinkedBlockingQueue<Command>();
	}
	
	public synchronized void start() {
		this.pcih = new LineInputHandler(pcin, this.queue);
		this.btih = new CommandInputHandler(dis, this.queue);
		this.pcthread = new Thread(pcih, "PC Input Handler");
		this.btthread = new Thread(btih, "BT Input Handler");
		this.pcthread.start();
		this.btthread.start();
	}
	
	public synchronized void stop() {
		this.pcih.stop();
		this.btih.stop();
		this.pcthread.interrupt();
		this.pcthread.interrupt();
		try {
			/* Make sure thread is not stuck on blocked io */
			this.pcthread.join(1000);
		} catch (InterruptedException e) {}
		try {
			/* Make sure thread is not stuck on blocked io */
			this.btthread.join(1000);
		} catch (InterruptedException e) {}
		
		try {
			dis.close();
			dos.close();
		} catch (IOException ioe) {
			System.out.println("IOException closing connection:");
			System.out.println(ioe.getMessage());
		}
		
	}
	
	public synchronized int handleStdinCommand(String line) {
		int[] cmd = null;
		int ret = 0;
		if (0 == line.compareTo("quit")) {
			cmd = SLAMCommand.createHostDisconnect();
			ret = -1;
		}
			
		if (line.equalsIgnoreCase("connect")) {
			cmd = SLAMCommand.createHostConnect(1);
		}
		if (line.startsWith("set ")) {
			String[] strings = line.split(" ");
			if (strings.length == 4) {
				//System.out.printf("match: %s\n", line);
				try {
					int wheelA = Integer.parseInt(strings[1]);
					int wheelB = Integer.parseInt(strings[2]);
					int sensorPos = Integer.parseInt(strings[3]);
					cmd = SLAMCommand.createHostSet(wheelA, wheelB, sensorPos);
				} catch (NumberFormatException e) {
					System.out.println("Invalid format, all parameters must be ints\n");
				}
			} else {
				System.out.println("Invalid format, usage:\nset speedA speedB sensorPos\n");
			}
		}
		if (cmd != null) {
			try {
				for (int i = 0; i < cmd.length; i++) {
					dos.writeInt(cmd[i]);
				}
				dos.flush();
			} catch (IOException e) {
				e.printStackTrace();
				return -2;
			}
		}
		//System.out.printf("echo: %s\n", line);
		return ret;
	}
	
	public synchronized int handleBTCommand(int[] data) {
		switch (data[0]) {
		case SLAMCommand.CMD_ID_NXT_CONNECT:
			System.out.printf("Protocol version: %d\n", data[1]);
			break;
		case SLAMCommand.CMD_ID_NXT_MEASURE:
			System.out.printf("Measure: %d %d %d %d\n",
					data[1], data[2], data[3], data[4]);
			break;
		case SLAMCommand.CMD_ID_NXT_UPDATE:
			System.out.printf("Update: %d %d\n", data[1], data[2]);
			break;
		case SLAMCommand.CMD_ID_NXT_DISCONNECT:
			System.out.printf("disconnect\n");
			return -1;
		}
		return 0;
	}
	
	public synchronized int processCommands() {
		int ret = 0;
		while (ret >= 0) {
			try {
				Command cmd = this.queue.take();
				ret = cmd.execute(this);
			} catch (InterruptedException e) {
				/* Continue */
			}
		}
		return ret;
	}
	
	private DataOutputStream dos;
	private DataInputStream dis;
	private DataInputStream pcin;
	
	private LineInputHandler pcih;
	private CommandInputHandler btih;
	
	private Thread pcthread;
	private Thread btthread;
	private BlockingQueue<Command> queue;
}
