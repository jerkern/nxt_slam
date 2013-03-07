package se.ajn.slam.pchost;

import java.io.*;
import java.util.concurrent.*;

public class LineInputHandler implements Runnable {
	public LineInputHandler(DataInput in, BlockingQueue<Command> queue)
	{
		this.stop = false;
		this.in = in;
		this.queue = queue;
	}
	
	public void run()
	{
		String line = null;
		/* We can't hold the look for the entire loop,
		 * then it would be impossible to update the this.stop
		 * variable using this stop() method from another thread
		 */
		while (true) {
		
			synchronized (this) {
				if (this.stop)
					break;
			}
			
			if (line == null) {
				try {
					line = in.readLine();
					
				} catch (IOException e) {
					/* Restart loop */
					continue;
				}
			}
			
			StdinCommand cmd = new StdinCommand(line);
			if (line != null) {
				try {
					synchronized (this) {
						this.queue.put(cmd);
					}
					line = null;
				} catch (InterruptedException e) { /* Do nothing */ }
			}
		}
	}
	
	public synchronized void stop()
	{
		this.stop = true;
		notifyAll();
	}
	
	private boolean stop;
	private DataInput in;
	private BlockingQueue<Command> queue;
}
