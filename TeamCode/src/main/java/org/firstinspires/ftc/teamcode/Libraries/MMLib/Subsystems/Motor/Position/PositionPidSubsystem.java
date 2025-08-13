package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Position;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Base.PidBaseSubsystem;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMUtils;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.Set;
import java.util.function.DoubleSupplier;

import Ori.Coval.Logging.Logger.KoalaLog;


public class PositionPidSubsystem extends PidBaseSubsystem {

    public PositionPidSubsystem(String subsystemName) {
        super(subsystemName);
    }


    /**
     * Creates a Command that keeps the mechanism in place using PID control.
     *
     * @return a Command requiring this subsystem
     */
    @Override
    public Command getToAndHoldSetPointCommand(DoubleSupplier setPoint) {
        return new Command() {
            @Override
            public void initialize() {
                // clear previous errors/integral
                pidController.reset();
                pidController.setSetpoint(setPoint.getAsDouble());

                KoalaLog.log(subsystemName + "/pid setpoint", setPoint.getAsDouble(), true);
            }

            @Override
            public void execute() {
                pidController.setSetpoint(setPoint.getAsDouble());
                KoalaLog.log(subsystemName + "/pid setpoint", pidController.getSetpoint(), true);

                double pidOutput = KoalaLog.log(subsystemName + "/pid output", pidController.calculate(getPose()), true);
                setPower(pidOutput);// apply computed power
            }

            @Override
            public Set<Subsystem> getRequirements() {
                // Declare that this command requires the enclosing subsystem instance
                return Set.of(PositionPidSubsystem.this);
            }
        };
    }

    /**
     * Sets the acceptable position error tolerance for the PID setpoint.
     *
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public PositionPidSubsystem withPositionTolerance(double tolerance) {
        pidController.setTolerance(tolerance, pidController.getErrorRateTolerance());
        return this;
    }

    /**
     * Sets the acceptable velocity error tolerance for the PID setpoint.
     *
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public PositionPidSubsystem withVelocityTolerance(double tolerance) {
        pidController.setTolerance(pidController.getErrorTolerance(), tolerance);
        return this;
    }

    private DoubleSupplier debugKpSupplier;
    private DoubleSupplier debugKiSupplier;
    private DoubleSupplier debugKdSupplier;
    private DoubleSupplier debugIZoneSupplier;
    private DoubleSupplier debugPositionToleranceSupplier;
    private DoubleSupplier debugVelocityToleranceSupplier;
    private DoubleSupplier debugIntegralMinRangeSupplier;
    private DoubleSupplier debugIntegralMaxRangeSupplier;

    /**
     * add suppliers that when changed will auto update the pid values.
     * any value you don't need just put null
     *
     * @param debugKpSupplier                Kp
     * @param debugKiSupplier                Kd
     * @param debugKdSupplier                Ki
     * @param debugIZoneSupplier             iZone
     * @param debugPositionToleranceSupplier position tolerance
     * @param debugVelocityToleranceSupplier velocity tolerance
     * @param debugIntegralMinRangeSupplier    integral min range
     * @param debugIntegralMaxRangeSupplier    integral max range
     * @implNote !NOTICE THIS ONLY WORKS IF IN DEBUG MODE
     */
    public PidBaseSubsystem withDebugPidSuppliers(DoubleSupplier debugKpSupplier,
                                                  DoubleSupplier debugKiSupplier,
                                                  DoubleSupplier debugKdSupplier,
                                                  DoubleSupplier debugIZoneSupplier,
                                                  DoubleSupplier debugPositionToleranceSupplier,
                                                  DoubleSupplier debugVelocityToleranceSupplier,
                                                  DoubleSupplier debugIntegralMinRangeSupplier,
                                                  DoubleSupplier debugIntegralMaxRangeSupplier) {

        this.debugKpSupplier = debugKpSupplier;
        this.debugKiSupplier = debugKiSupplier;
        this.debugKdSupplier = debugKdSupplier;
        this.debugIZoneSupplier = debugIZoneSupplier;
        this.debugPositionToleranceSupplier = debugPositionToleranceSupplier;
        this.debugVelocityToleranceSupplier = debugVelocityToleranceSupplier;
        this.debugIntegralMinRangeSupplier = debugIntegralMinRangeSupplier;
        this.debugIntegralMaxRangeSupplier = debugIntegralMaxRangeSupplier;

        return this;
    }


    @Override
    public void periodic() {
        if (MMRobot.getInstance().currentOpMode != null &&
                MMRobot.getInstance().currentOpMode.opModeType == OpModeType.NonCompetition.DEBUG) {

            MMUtils.updateIfChanged(
                    debugKpSupplier,
                    pidController::getP,
                    pidController::setP
            );
            MMUtils.updateIfChanged(
                    debugKiSupplier,
                    pidController::getI,
                    pidController::setI
            );
            MMUtils.updateIfChanged(
                    debugKdSupplier,
                    pidController::getD,
                    pidController::setD
            );
            MMUtils.updateIfChanged(
                    debugIZoneSupplier,
                    pidController::getIZone,
                    pidController::setIZone
            );
            MMUtils.updateIfChanged(
                    debugPositionToleranceSupplier,
                    pidController::getErrorTolerance,
                    this::withPositionTolerance
            );
            MMUtils.updateIfChanged(
                    debugVelocityToleranceSupplier,
                    pidController::getErrorRateTolerance,
                    this::withVelocityTolerance
            );

            MMUtils.updateIfChanged(
                    debugIntegralMinRangeSupplier,
                    pidController::getMinimumIntegral,
                    this::withMinIntegralRange
            );

            MMUtils.updateIfChanged(
                    debugIntegralMaxRangeSupplier,
                    pidController::getMaximumIntegral,
                    this::withMaxIntegralRange
            );
        }
    }
}
