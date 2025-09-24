package org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;

/**
 * Rotary encoder connected through a motor encoder port
 * */
public class CuttleEncoder
{
    public CuttleRevHub hub;
    public final double encTicks;
    private int direction = 1;
    public int mPort;
    private double offsetTicks = 0;

    /**
     * @param revHub
     * @param port Motor port of the encoder
     * @param countsPerRevolution Number of counts per revolution of the encoder
     * */
    public CuttleEncoder(CuttleRevHub revHub, int port, double countsPerRevolution) {
        hub = revHub;
        encTicks = countsPerRevolution;
        mPort = port;
        offsetTicks = getCounts();
    }

    /**
     * @param revHub
     * @param port Motor port of the encoder
     * @param countsPerRevolution Number of counts per revolution of the encoder
     * */
    public CuttleEncoder(CuttleRevHub revHub, int port, double countsPerRevolution, Direction direction) {
        this(revHub, port, countsPerRevolution);
        setDirection(direction);
    }

    /**
     *
     * @return the velocity in rpm
     */
    public double getVelocity()
    {
        return hub.bulkData.getEncoderVelocity(mPort)/encTicks*direction;
    }

    /**
     * Get the number of counts that the encoder has turned
     * */
    public double getCounts() {
        return (hub.bulkData.getEncoderPosition(mPort) - offsetTicks)*direction;
    }

    public double getPose(){
        return getCounts()/encTicks;
    }

    public void setPose(double pose){
        offsetTicks -= getCounts() - pose;
    }

    /**
     * Set the direction of the encoder.
     * @param direction
     * */
    public CuttleEncoder setDirection(Direction direction)
    {
        if(direction == Direction.REVERSE)
        {
            this.direction = -1;
        }
        else
        {
            this.direction = 1;
        }

        return this;
    }
}
