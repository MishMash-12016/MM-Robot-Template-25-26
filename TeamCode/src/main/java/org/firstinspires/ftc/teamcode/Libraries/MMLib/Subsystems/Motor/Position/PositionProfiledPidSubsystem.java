package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Position;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.tuning.FFKsSysid;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.tuning.FFKvSysid;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Base.ProfiledPidBase;

import java.util.Set;
import java.util.function.DoubleSupplier;

import Ori.Coval.Logging.Logger.KoalaLog;

public class PositionProfiledPidSubsystem extends ProfiledPidBase {
    public PositionProfiledPidSubsystem(String subsystemName) {
        super(subsystemName);
    }


    /**
     * Creates a Command that keeps the mechanism in place using PID control.
     *
     * @return a Command requiring this subsystem
     */
    //this command is the base for all the other pid commands
    public Command getToAndHoldSetPointCommand(DoubleSupplier setPoint) {
        return new Command() {
            @Override
            public void initialize() {
                // clear previous errors/integral
                profiledPIDController.reset(getPose(), getVelocity());
                profiledPIDController.setGoal(setPoint.getAsDouble());

                KoalaLog.log(subsystemName + "/pid setpoint", setPoint.getAsDouble(), true);
            }

            @Override
            public void execute() {
                double pidOutput = KoalaLog.log(
                        subsystemName + "/pid output",
                        profiledPIDController.calculate(getPose(), setPoint.getAsDouble()),
                        true);

                double feedforwardOutput = KoalaLog.log(
                        subsystemName + "/feedforward output",
                        feedforward.calculate(profiledPIDController.getSetpoint().velocity),
                        true);//TODO: check whether to divide here by batteryVoltage

                setPower(pidOutput + feedforwardOutput);// apply computed power
            }

            @Override
            public Set<Subsystem> getRequirements() {
                // Declare that this command requires the enclosing subsystem instance
                return Set.of(PositionProfiledPidSubsystem.this);
            }
        };
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
        return getToAndHoldSetPointCommand(()->setPoint).interruptOn(this::getAtGoal);
    }

    /**
     * Creates a Command that keeps the mechanism in its current setpoint place using PID control.
     *
     * @return a Command requiring this subsystem
     */
    public Command holdCurrentSetPointCommand() {
        return getToAndHoldSetPointCommand(()-> profiledPIDController.getGoal().position);
    }

    public Command tuneKSCommand(double rampRate, double minVelocity){
        return new FFKsSysid(rampRate,minVelocity,this,this::setPower,this::getVelocity);
    }

    public Command tuneKVCommand(double rampRate, double kS){
        return new FFKvSysid(rampRate, kS, 5, this, this::setPower, this::getVelocity);
    }
}
