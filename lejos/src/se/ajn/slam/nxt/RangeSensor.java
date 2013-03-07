package se.ajn.slam.nxt;
import lejos.nxt.I2CPort;
import lejos.robotics.RangeFinder;

public class RangeSensor extends PCF8591 implements RangeFinder
{
    /** Device address of the chip */
    private static int DEFAULT_I2C_ADDRESS = 0x90;
    
    /** Device channel of the NXT */
    private static final byte CHANNEL = 0;
    /**
     * Constructor
     * @param i2cPort Port where the chip is attached to (e.g. SensorPort.S1)
     */
    public RangeSensor(I2CPort port)
    {
	super(port, DEFAULT_I2C_ADDRESS, (byte) 0);
    }

    /**
     * Read a distance from input
     * @return value  to be returned
     */	
    public float getRange()
    {
    	/* Calibrated 2011-12-14 */
    	return (float) (4.56*getDistance()-0.69);
    }
     
    /**
     * Read a range vector from input
     * @return value to be returned
     * @TODO Implement
     */
    public float[] getRanges()
    {
    	return null;
    }
   
    /**
     * Read a distance from input
     * @return value  to be returned
     */
    public int getDistance()
    {
	return super.read(CHANNEL) & 0xFF;
    }
}
