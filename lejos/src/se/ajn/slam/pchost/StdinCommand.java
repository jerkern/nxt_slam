package se.ajn.slam.pchost;

public class StdinCommand implements Command {
	public StdinCommand(String str) {
		this.line = str;
	}
	public int execute(PCBridge host) {
		return host.handleStdinCommand(this.line);
	}
	private String line;
}
