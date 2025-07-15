package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Base;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleDigital;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.PIDController;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import Ori.Coval.Logging.Logger.KoalaLog;


public class PidBaseSubsystem extends MotorOrCrServoSubsystem {
    // Encoder that measures current position and velocity (ticks converted via ratio)
    private CuttleEncoder encoder;
    public PIDController pidController = new PIDController(0, 0, 0);
    public SimpleMotorFeedforward feedforward;

    //base
    public PidBaseSubsystem(String subsystemName) {
        super(subsystemName);
    }

    /**
     * Creates a Command that keeps the mechanism in place using PID control.
     *
     * @return a Command requiring this subsystem
     */
    //this command is the base for all the other pid commands
    public Command getToAndHoldSetPointCommand(DoubleSupplier setPoint) {
        return new RunCommand(()-> {
            KoalaLog.log(subsystemName + "/ERROR ", "pid holdSetPointCommand is not implemented", true);
            setPower(0);}, this);
    }

    /**
     * Creates a Command that keeps the mechanism in place using PID control.
     *
     * @return a Command requiring this subsystem
     */
    public Command getToAndHoldSetPointCommand(double setPoint) {
        return getToAndHoldSetPointCommand(()->setPoint);
    }

    /**
     * Creates a Command that moves the mechanism to the specified setpoint using PID control.
     *
     * @param setPoint target setpoint (in encoder units, adjusted by ratio)
     * @return a Command requiring this subsystem
     */
    public Command getToSetpointCommand(double setPoint) {
        return getToAndHoldSetPointCommand(()->setPoint).interruptOn(()->KoalaLog.log(subsystemName + "/atSetpoint", pidController.atSetpoint(), true));
    }

    /**
     * Creates a Command that keeps the mechanism in its current setpoint place using PID control.
     *
     * @return a Command requiring this subsystem
     */
    public Command holdCurrentSetPointCommand() {
        return getToAndHoldSetPointCommand(()->pidController.getSetpoint());
    }

    /**
     * Returns the current position (pose) provided by the encoder.
     *
     * @return current pose in encoder units (divided by ratio)
     */
    public double getPose() {
        return KoalaLog.log(subsystemName + "/pose", encoder.getPose(), true);
    }

    public double getVelocity() {
        return KoalaLog.log(subsystemName + "/velocity", encoder.getVelocity(), true);
    }

    public void setPose(double pose) {
        encoder.setPose(pose);
    }

    public PidBaseSubsystem withEncoder(CuttleRevHub revHub, int encoderPort, double cpr, Direction direction) {
        encoder = new CuttleEncoder(revHub, encoderPort, cpr, direction);
        return this;
    }

    /**
     * Configures a zero-position limit switch that resets encoder when activated.
     *
     * @param zeroSwitch digital switch input
     * @param zeroPose   encoder value to set when switch is pressed
     * @return this subsystem for chaining
     */
    public PidBaseSubsystem withZeroSwitch(CuttleDigital zeroSwitch, double zeroPose) {
        new Trigger(zeroSwitch::getState)
                .whenActive(() -> encoder.setPose(zeroPose));
        return this;
    }

    /**
     * Configures a zero-position limit switch that resets encoder when activated.
     *
     * @param zeroSupplier a supplier of when to zero the system
     * @param zeroPose     encoder value to set when switch is pressed
     * @return this subsystem for chaining
     */
    public PidBaseSubsystem withZeroSupplier(BooleanSupplier zeroSupplier, double zeroPose) {
        new Trigger(zeroSupplier)
                .whenActive(() -> encoder.setPose(zeroPose));
        return this;
    }

    /**
     * Updates PID gains.
     *
     * @param kp proportional gain
     * @param ki integral gain
     * @param kd derivative gain
     * @return this subsystem for chaining
     */
    public PidBaseSubsystem withPid(double kp, double ki, double kd) {
        pidController.setPID(kp, ki, kd);
        return this;
    }


    /**
     * Defines the Integral Zone (I-Zone) for the PID controller.
     *
     * @param iZone range around setpoint where integral is active
     * @return this subsystem for chaining
     */
    public PidBaseSubsystem withIZone(double iZone) {
        pidController.setIZone(iZone);
        return this;
    }

    /**
     * Restricts the integral accumulator within given bounds.
     *
     * @param minIntegralRange minimum accumulator value
     * @param maxIntegralRange maximum accumulator value
     * @return this subsystem for chaining
     */
    public PidBaseSubsystem withIntegralRange(double minIntegralRange, double maxIntegralRange) {
        pidController.setIntegratorRange(minIntegralRange, maxIntegralRange);
        return this;
    }

    /**
     * Restricts the integral accumulator within given bounds.
     *
     * @param minIntegralRange minimum accumulator value
     * @return this subsystem for chaining
     */
    public PidBaseSubsystem withMinIntegralRange(double minIntegralRange) {
        return withIntegralRange(minIntegralRange, pidController.getMinimumIntegral());
    }

    /**
     * Restricts the integral accumulator within given bounds.
     *
     * @param maxIntegralRange minimum accumulator value
     * @return this subsystem for chaining
     */
    public PidBaseSubsystem withMaxIntegralRange(double maxIntegralRange) {
        return withIntegralRange(pidController.getMinimumIntegral(), maxIntegralRange);
    }

    @Override
    public void resetHub() {
        super.resetHub();
        double pose = getPose();
        if(encoder.hub.getHubName().equals(MMRobot.getInstance().controlHub.getHubName())){
            encoder = new CuttleEncoder(MMRobot.getInstance().controlHub, encoder.mPort, encoder.encTicks);
        }
        else {
            encoder = new CuttleEncoder(MMRobot.getInstance().expansionHub, encoder.mPort, encoder.encTicks);
        }
        setPose(pose);
    }
}
