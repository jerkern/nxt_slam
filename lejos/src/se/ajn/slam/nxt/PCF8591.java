package se.ajn.slam.nxt;
import lejos.nxt.I2CPort;


public class PCF8591 {
    
    /** Port where the chip is attached to */
    private final I2CPort port;
    
    /** Device address of the chip */
    private final int address;

    /** Buffer for i/o */
    private byte buffer[] = {0,0,0,0,0};

    /** Operational mode */
    private byte mode;

    /** Standard mode: four single-ended inputs */
    public static final byte MODE_STANDARD=0;

    /** Three differential mode: input 3 gets subtracted from input 0,1 and 2 */
    public static final byte MODE_THREE_DIFFERENTIAL=1;

    /** Mixed mode: input 0 and 1 are single-ended inputs, input 3 gets subtracted from input 2 */
    public static final byte MODE_MIXED=2;

    /** Dual differential mode: input 1 is subtracted from input 0, input 3 is subtracted fom input 2 */
    public static final byte MODE_DUAL_DIFFERENTIAL=3;

    /**
     * Constructor
     * @param i2cPort Port where the chip is attached to (e.g. SensorPort.S1)
     * @param deviceAddress Device address of the chip (e.g. 0x90)
     * @param mode Operational mode of the analog inputs, see MODE_XXX constants
     */
    public PCF8591(I2CPort i2cPort, int deviceAddress, byte mode) {
        port = i2cPort;
        address = deviceAddress;
        this.mode = MODE_STANDARD;
        port.i2cEnable(I2CPort.STANDARD_MODE);
    }

    /**
     * Write a byte to the analog output
     * @param value Data to be written
     */
    public void write(byte value) {
        buffer[0] = (byte) (0x40+(mode<<4));
        buffer[1] = value;
        port.i2cTransaction(address,buffer,0,2,null,0,0);
    }

    /**
     * Read from a single analog input
     * @param channel Channel number (0-3)
     * @return Data from the A/D converter
     */
    public byte read(byte channel) {
        buffer[0] = (byte) (0x40+(mode<<4)+channel);
        port.i2cTransaction(address,buffer,0,1,buffer,0,2);
        return buffer[1];
    }

    /**
     * Read from all analog inputs
     * @return Data from the A/D converter, 4 bytes
     */
    public byte[] read() {
        buffer[0] = (byte) (0x44+(mode<<4));
        port.i2cTransaction(address,buffer,0,1,buffer,0,5);
        byte[] result=new byte[4];
        result[0] = buffer[1];
        result[1] = buffer[2];
        result[2] = buffer[3];
        result[3] = buffer[4];
        return result;
    }

}
