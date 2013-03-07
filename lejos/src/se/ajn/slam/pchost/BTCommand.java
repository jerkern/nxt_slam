package se.ajn.slam.pchost;

public class BTCommand implements Command {
	public BTCommand(int[] raw) {
		this.data = raw.clone();
	}
	public int execute(PCBridge host) {
		return host.handleBTCommand(data);
	}
	private int[] data;
}
