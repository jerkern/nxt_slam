package se.ajn.slam.nxt;
import lejos.nxt.*;
import lejos.util.Timer;
import lejos.util.TimerListener;
import java.io.*;
import se.ajn.slam.SLAMCommand;

/**
 * SLAM Robot
 **/
public class SLAMRobot implements TimerListener {
	private static final int samplePeriod = 500; // ms
	public SLAMRobot()
	{
	    //	this.sensor = new UltrasonicSensor(SensorPort.S4);
	    this.sensor = new RangeSensor(SensorPort.S4);
		this.motorA = new NXTRegulatedMotor(MotorPort.C);
		this.motorB = new NXTRegulatedMotor(MotorPort.B);
		this.motorSensor = new NXTRegulatedMotor(MotorPort.A);
		this.timer = new Timer(SLAMRobot.samplePeriod, this);
		this.ch = null;
		this.motorA.flt(true);
		this.motorB.flt(true);
		this.motorSensor.setSpeed(100);
		this.motorSensor.flt(true);
	}
	
	
	public synchronized int handleCommand(int[] cmd) 
	{	
		LCD.clear();
		LCD.drawInt(cmd[0],0,0);
		LCD.refresh();
		switch (cmd[0]) {
		case SLAMCommand.CMD_ID_HOST_SET:
			this.motorA.setSpeed(Math.abs(cmd[1]));
			this.motorB.setSpeed(Math.abs(cmd[2]));
			this.motorSensor.rotateTo(cmd[3], true);
			if (cmd[1] > 0) {
				this.motorA.forward();
			} else {
				this.motorA.backward();
			}
			if (cmd[2] > 0) {
				this.motorB.forward();
			} else {
				this.motorB.backward();
			}
			break;
		case SLAMCommand.CMD_ID_HOST_CONNECT:
			/* This is unexpected after initial connection */
			break;
		default:
			// All other commands are interpreted as a disconnect
		case SLAMCommand.CMD_ID_HOST_DISCONNECT:
			this.motorA.flt(true);
			this.motorB.flt(true);
			this.motorSensor.flt(true);
			return -1;
		}
		return 0;
	}
	
	public synchronized void timedOut()
	{
		/* Called periodically by timer, triggers new sensor measurement */
		int wheelA = this.motorA.getTachoCount();
		int wheelB = this.motorB.getTachoCount();
		int sensor = this.motorSensor.getTachoCount();
		int dist = (int) this.sensor.getRange();
		
		int[] msg = SLAMCommand.createMeasure(wheelA,wheelB,dist,sensor);
	    try {
	    	ch.writeMessage(msg);
	    } catch (IOException e) {}
		return;
	}
	
	public synchronized void startPeriodic() {
		this.timer.start();
	}
	
	public synchronized void setCommHandler(CommHandler ch) {
		this.ch = ch;
	}
	
	public static void main(String [] args)  throws Exception 
	{
        LCD.clear();
		LCD.drawString("Waiting...",0,0);
		LCD.refresh();

		SLAMRobot robot = new SLAMRobot();
		CommHandler ch = new CommHandler(robot);
		robot.setCommHandler(ch);
		
		Thread commThread = new Thread(ch);
		commThread.start();
		
		/* Wait for established connection */
		ch.waitConnect();
		
		LCD.clear();
		LCD.drawString("Connected",0,0);
		LCD.refresh();
		
		/* Star timer */
		robot.startPeriodic();

		/* Wait for com thread to return, then close down */
		 
		//commThread.join(); 
		/* not yet implemented in LEJOS... 
		 * ugly work-around instead */
		while (commThread.isAlive()) {
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {}
		}
		
		LCD.clear();
		LCD.drawString("Closing...",0,0);
		LCD.refresh();
	}

	private UltrasonicSensor sensor;
	private NXTRegulatedMotor motorA;
	private NXTRegulatedMotor motorB;
	private NXTRegulatedMotor motorSensor;
	private Timer timer;
	private CommHandler ch;
	
}