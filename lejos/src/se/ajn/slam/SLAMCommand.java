package se.ajn.slam;
import java.io.*;

/**
 * 
 * Protocol version 1
 * 
 * Message, all start with 0xF00F {length} {id} and end with 0x0FF0, length is
 * number of ints including header/footer
 * 
 * NXT -> Host:
 * connect: 	0xF00F 0x5 0x1 {version} 0x0FF0
 * disconnect: 	0xF00F 0x4 0x2 0x0FF0 
 * measure: 	0xF00F 0x8 0x3 {wheelA_encoder_value} {wheelB_encoder_value} {sensor_encoder_value} {sensor_value} 0x0FF0
 * update: 		0xF00F 0x6 0x4 {wheelA_encoder_value} {wheelB_encoder_value} 0x0FF0
 * 
 * Host -> NXT
 * connect: 	0xF00F 0x5 0x11 {version} 0x0FF0
 * disconnect: 	0xF00F 0x4 0x12 0x0FF0
 * set: 		0XF00F 0x7 0x13 {wheelA_speed} {wheelB_speed} {desired_sensor_position} 0x0FF0
 * 
 */
public class SLAMCommand {
	public static int[] readCommand(DataInputStream inputStream) throws IOException {
		int[] msg = SLAMCommand.readMessage(inputStream);
		int[] cmd = new int[msg[1] - SLAMCommand.HDR_SIZE];
		
		for (int i = 0; i < cmd.length; i++) {
			cmd[i] = msg[i+2];
		}
		return cmd;
	}
	
	public static int[] createMeasure(int wheelA, int wheelB,
											int sensorVal, int sensorPos) {
		int[] cmd = new int[5];
		cmd[0] = SLAMCommand.CMD_ID_NXT_MEASURE;
		cmd[1] = wheelA;
		cmd[2] = wheelB;
		cmd[3] = sensorPos;
		cmd[4] = sensorVal;
		return addHeader(cmd);
	}

	public static int[] createUpdat(int wheelA, int wheelB) {
		int[] cmd = new int[3];
		cmd[0] = SLAMCommand.CMD_ID_NXT_UPDATE;
		cmd[1] = wheelA;
		cmd[2] = wheelB;
		return addHeader(cmd);
	}
	
	public static int[] createHostConnect(int version) {
		int[] cmd = new int[2];
		cmd[0] = SLAMCommand.CMD_ID_HOST_CONNECT;
		cmd[1] = version;
		return addHeader(cmd);
	}
	public static int[] createHostDisconnect() {
		int[] cmd = new int[1];
		cmd[0] = SLAMCommand.CMD_ID_HOST_DISCONNECT;
		return addHeader(cmd);
	}
	public static int[] createHostSet(int wheelA, int wheelB, int sensorPos) {
		int[] cmd = new int[4];
		cmd[0] = SLAMCommand.CMD_ID_HOST_SET;
		cmd[1] = wheelA;
		cmd[2] = wheelB;
		cmd[3] = sensorPos;
		return addHeader(cmd);
	}
	
	public static int[] createNXTConnect(int version) {
		int[] cmd = new int[2];
		cmd[0] = SLAMCommand.CMD_ID_NXT_CONNECT;
		cmd[1] = version;
		return addHeader(cmd);
	}
	
	private static int[] readMessage(DataInputStream inputStream) throws IOException {
		/* Reserve enough space for any message */
		int[] msg = new int[SLAMCommand.MAX_LENGTH];
		int tmp;
		/* Look for start byte, ignoring any leading garbage */
		do {
			tmp = inputStream.readInt();
		} while (tmp != SLAMCommand.CMD_START);
		msg[0] = tmp;
		/* Read size */
		tmp = inputStream.readInt();
		if (tmp <= SLAMCommand.MAX_LENGTH && tmp >= SLAMCommand.MIN_LENGTH) {
			msg[1] = tmp;
		} else {
			throw new IOException("bad (SLAM) message size");
		}
		for (int i = 2; i < msg[1]; i++) {
			msg[i] = inputStream.readInt();
		}
		int lastInd = msg[1] - 1;
		if (msg[lastInd] != SLAMCommand.CMD_STOP) {
			/* Bad message */
			throw new IOException("bad (SLAM) message structure");
		}
		return msg;
	}

	private static int[] addHeader(int[] cmd) {
		int[] msg = new int[cmd.length+SLAMCommand.HDR_SIZE];
		msg[0] = SLAMCommand.CMD_START;
		msg[1] = cmd.length+SLAMCommand.HDR_SIZE;
		msg[msg.length-1] = SLAMCommand.CMD_STOP;
		System.arraycopy(cmd, 0, msg, 2, cmd.length);
		return msg;
	}
	
	public static final int MAX_LENGTH = 0x8;
	private static final int MIN_LENGTH = 0x3;
	private static final int CMD_START = 0xF00F;
	private static final int CMD_STOP = 0xFF0;
	private static final int HDR_SIZE = 0x3;
	
	public static final int CMD_ID_NXT_CONNECT = 0x1;
	public static final int CMD_ID_NXT_DISCONNECT = 0x2;
	public static final int CMD_ID_NXT_MEASURE = 0x3;
	public static final int CMD_ID_NXT_UPDATE = 0x4;
	
	public static final int CMD_ID_HOST_CONNECT = 0x11;
	public static final int CMD_ID_HOST_DISCONNECT = 0x12;
	public static final int CMD_ID_HOST_SET = 0x3;
}
