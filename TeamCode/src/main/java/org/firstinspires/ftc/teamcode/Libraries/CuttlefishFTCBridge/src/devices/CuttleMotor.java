package org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMUtils;


/**
 * Cuttlefish DCMotor implementation.
 */
public class CuttleMotor {
    public CuttleRevHub hub;
    public int mPort;
    int sign = 1;
    double power;
    public boolean interlaced;

    //TODO: find nominal voltag
    double nominalVoltage = 12;

    /**
     * @param revHub the control/expension hub
     * @param port   the port it is connected to on the control/expension hub
     */
    public CuttleMotor(CuttleRevHub revHub, int port) {
        hub = revHub;
        mPort = port;
    }

    /**
     * @param revHub the control/expension hub
     * @param port   the port it is connected to on the control/expension hub
     */
    public CuttleMotor(CuttleRevHub revHub, int port, Direction direction) {
        this(revHub, port);
        setDirection(direction);
    }

    /**
     * @param power the power to set the motor to between -1 to 1
     */
    public void setPower(double power) {
        this.power = voltageCompensate(power);

        if (!interlaced) {
            sendPower();
        }
    }


    // Tweak α to control smoothing (0.0 = full freeze, 1.0 = no smoothing)
    private static final double EMA_ALPHA = 0.1;

    // Holds the “last” filtered voltage. Initialize it once (e.g. in init()).
    private double filteredVoltage = 0.0;

    /**
     * compensates for battery voltage
     *
     * @param uncompensatedPower The “ideal” power you want (–1.0…+1.0),
     *                           as if battery were always at nominalVoltage.
     * @return The scaled/clamped power (–1.0…+1.0) that compensates
     * for batterys̈ag.
     */
    private double voltageCompensate(double uncompensatedPower) {
        // 1) Read the raw battery voltage once per loop
        double rawVoltage = hub.getBatteryVoltage() / 1000.0;

        // 2) Initialize filteredVoltage on first call (if still 0.0).
        //    You could also set filteredVoltage = hub.getBatteryVoltage() in init().
        if (filteredVoltage <= 0.0) {
            filteredVoltage = rawVoltage;
        }

        // 3) EMA update: filteredVoltage = α·(new reading) + (1–α)·(old filtered)
        filteredVoltage = EMA_ALPHA * rawVoltage + (1.0 - EMA_ALPHA) * filteredVoltage;

        // 4) Compute the “ideal” fraction: (uncompensatedPower·nominalVoltage)/filteredVoltage
        double scaled = (uncompensatedPower * nominalVoltage) / filteredVoltage;

        // 5) Explicitly clamp to ±1.0 so we never request more than 100% from the controller\
        return MMUtils.clamp(scaled, -1.0, 1.0);
    }


    /**
     * MishMash added
     *
     * @return power sent to motor
     */
    public double getPower() {
        return sign * power;
    }


    /**
     * Send cached motor power to the hub.
     * <br>
     * This is not necessary and should not be used under ordinary conditions.
     */
    public void sendPower() {
        hub.setMotorPower(mPort, sign * power);
    }


    /**
     * Set the direction of the motor
     *
     * @param direction the direction to move the motor
     */
    public CuttleMotor setDirection(@NonNull Direction direction) {
        if (direction == Direction.FORWARD) {
            sign = 1;
        } else {
            sign = -1;
        }
        return this;
    }

    /**
     * Get motor current in milli-amps.
     * <br>
     * WARNING: This will poll the hub an extra time costing about 3ms.
     *
     * @return Motor current in milli-amps
     */
    public int getCurrent() {
        return this.hub.getMotorCurrent(this.mPort);
    }

    /**
     * Set the zero power behaviour of the motor.
     *
     * @param behaviour choose if the motor should break or coast
     */
    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour) {
        this.hub.setMotorZeroPowerBehaviour(this.mPort, behaviour);
    }
}
