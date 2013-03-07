package se.ajn.slam.pchost;

import java.io.DataInputStream;
import java.io.IOException;
import java.util.concurrent.BlockingQueue;

import se.ajn.slam.SLAMCommand;

public class CommandInputHandler implements Runnable {
	public CommandInputHandler(DataInputStream in, BlockingQueue<Command> queue)
	{
		this.stop = false;
		this.in = in;
		this.queue = queue;
	}
	
	public void run()
	{
		int[] rawcmd = null;
		/* We can't hold the look for the entire loop,
		 * then it would be impossible to update the this.stop
		 * variable using this stop() method from another thread
		 */
		while (true) {
			synchronized (this) {
				if (this.stop)
					break;
			}
			
			if (rawcmd == null) {
				try {
					rawcmd = SLAMCommand.readCommand(this.in);
				} catch (IOException e) {
					/* Restart loop */
					continue;
				}
			}
			
			BTCommand cmd = new BTCommand(rawcmd);
			try {
				synchronized (this) {
					this.queue.put(cmd);
				}
				rawcmd = null;
			} catch (InterruptedException e) {
				/* Keep trying, unless we should stop */
				continue;
			}
		}
	}
	
	public synchronized void stop()
	{
		this.stop = true;
		notifyAll();
	}
	
	private boolean stop;
	private DataInputStream in;
	private BlockingQueue<Command> queue;
}
