package org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;

/**
 * Cuttlefish compatible cr servo.
 */
public class CuttleCrServo{

    private double power = 0.0;
    public int port;
    boolean enabled = false;
    final boolean FTCServo;

    Direction direction = Direction.FORWARD;
    public CuttleRevHub hub;
    com.qualcomm.robotcore.hardware.CRServo ftcServoDevice;

    /**
     * Initialize servo using cuttlefish direct access system
     * @param revHub
     * @param servoPort
     * */
    public CuttleCrServo(CuttleRevHub revHub, int servoPort, Direction direction)
    {
        port = servoPort;
        hub = revHub;
        FTCServo = false;
        this.direction = direction;
    }

    /**
     * Initialize servo using cuttlefish direct access system
     * @param revHub
     * @param servoPort
     * */
    public CuttleCrServo(CuttleRevHub revHub, int servoPort)
    {
        port = servoPort;
        hub = revHub;
        FTCServo = false;
    }

    /**
     * Initialize cr servo using hardwareMap
     * @param hardwareMap hardwareMap object
     * @param name Name of the servo in the config
     * */
    public CuttleCrServo(HardwareMap hardwareMap, String name, Direction direction)
    {
        FTCServo = true;
        ftcServoDevice = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class,name);
        this.direction = direction;
    }

    /**
     * Initialize cr servo using hardwareMap
     * @param hardwareMap hardwareMap object
     * @param name Name of the servo in the config
     * */
    public CuttleCrServo(HardwareMap hardwareMap, String name)
    {
        FTCServo = true;
        ftcServoDevice = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class,name);
    }

    /**
     * Set the target power of the servo
     * @param power Target power
     * */
    public void setPower(double power) {
        power = Math.max(-1, Math.min(1, power));
        this.power = direction == Direction.REVERSE ? -1 * power : power;

        if(!FTCServo)
        {
            //converting value from between -1 to 1 to 0 to 1
            double crPower = this.power/2 + 0.5;
            hub.setServoPosition(port,crPower);
            if(!enabled)
            {
                enablePWM(true);
            }
        }
        else
        {
            ftcServoDevice.setPower(this.power);
        }
    }

    /**
     * Enable or disable PWM on the servo port. This will not work if the servo was obtained using hardwareMap.
     * @param  enable If set to true PWM will be enabled, and if set to false PWM will be disabled
     * */
    public void enablePWM(boolean enable)
    {
        if(!FTCServo)
        {
            hub.enableServoPWM(port,enable);
            enabled = enable;
        }
    }

    /**
     * Get the target power of the servo.
     * <br>
     * IMPORTANT: This will not give the actual power of the servo.
     * */
    public double getPower() {
        return power;
    }

    public CuttleCrServo setDirection(Direction direction) {
        this.direction = direction;
        return this;
    }
}
