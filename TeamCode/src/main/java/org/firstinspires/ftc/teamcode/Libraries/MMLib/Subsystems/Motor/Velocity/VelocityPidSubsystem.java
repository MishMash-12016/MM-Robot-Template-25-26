package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Velocity;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.tuning.FFKsSysid;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.tuning.FFKvSysid;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Base.PidBaseSubsystem;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMUtils;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.MMRobot;
import java.util.Set;
import java.util.function.DoubleSupplier;

import Ori.Coval.Logging.Logger.KoalaLog;
/**
 * MotorVelocityPidSubsystem provides PID-controlled velocity for a motor-driven mechanism.
 *
 * <p>This subsystem uses a CuttleEncoder to measure velocity, a list of CuttleMotors to apply power,
 * and a PIDController to compute the required power to reach a target velocity setpoint.
 * </p>
 */
public class VelocityPidSubsystem extends PidBaseSubsystem {


    public VelocityPidSubsystem(String subsystemName) {
        super(subsystemName);
    }

    /**
     * @param feedforward the feedforward
     */
    public VelocityPidSubsystem withFeedForward(SimpleMotorFeedforward feedforward) {
        this.feedforward = feedforward;
        return this;
    }

    /**
     * Creates a Command that keeps the mechanism in place using PID control.
     *
     * @return a Command requiring this subsystem
     */
    @Override
    public Command getToAndHoldSetPointCommand(double setPoint) {
        return new Command() {
            @Override
            public void initialize() {
                // clear previous errors/integral
                pidController.reset();
                pidController.setSetpoint(setPoint);

                KoalaLog.log(subsystemName + "/pid setpoint", setPoint, true);
            }

            @Override
            public void execute() {
                double pidOutput = KoalaLog.log(subsystemName + "/pid output", pidController.calculate(getVelocity()), true);
                double feedforwardOutput = 0;


                if (feedforward != null) {
                    feedforwardOutput = feedforward.calculate(pidController.getSetpoint());
                }
                KoalaLog.log(subsystemName + "/pid feedforward", feedforwardOutput, true);

                setPower(pidOutput + feedforwardOutput);// apply computed power
            }

            @Override
            public Set<Subsystem> getRequirements() {
                // Declare that this command requires the enclosing subsystem instance
                return Set.of(VelocityPidSubsystem.this);
            }
        };
    }

    public Command tuneKSCommand(double rampRate, double minVelocity){
        return new FFKsSysid(rampRate,minVelocity,this,this::setPower,this::getVelocity);
    }

    public Command tuneKVCommand(double rampRate, double kS){
        return new FFKvSysid(rampRate, kS, 5, this, this::setPower, this::getVelocity);
    }

    /**
     * Updates feedforward gains.
     *
     * @param ks static gain
     * @param kv velocity gain
     * @param ka acceleration gain
     */
    public VelocityPidSubsystem withFeedforward(double ks, double kv, double ka) {
        return withFeedForward(new SimpleMotorFeedforward(ks, kv, ka));
    }

    /**
     * Sets tolerance for velocity error.
     */
    public VelocityPidSubsystem withVelocityTolerance(double tolerance) {
        pidController.setTolerance(tolerance);
        return this;
    }

    public VelocityPidSubsystem withAccelerationTolerance(double tolerance) {
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
    private DoubleSupplier debugKsSupplier;
    private DoubleSupplier debugKvSupplier;
    private DoubleSupplier debugKaSupplier;

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
     * @param debugKsSupplier                  static gain
     * @param debugKvSupplier                  velocity gain
     * @param debugKaSupplier                  acceleration gain
     * @implNote !NOTICE THIS ONLY WORKS IF IN DEBUG MODE
     */
    public PidBaseSubsystem withDebugPidSuppliers(DoubleSupplier debugKpSupplier,
                                                  DoubleSupplier debugKiSupplier,
                                                  DoubleSupplier debugKdSupplier,
                                                  DoubleSupplier debugIZoneSupplier,
                                                  DoubleSupplier debugPositionToleranceSupplier,
                                                  DoubleSupplier debugVelocityToleranceSupplier,
                                                  DoubleSupplier debugIntegralMinRangeSupplier,
                                                  DoubleSupplier debugIntegralMaxRangeSupplier,
                                                  DoubleSupplier debugKsSupplier,
                                                  DoubleSupplier debugKvSupplier,
                                                  DoubleSupplier debugKaSupplier) {

        this.debugKpSupplier = debugKpSupplier;
        this.debugKiSupplier = debugKiSupplier;
        this.debugKdSupplier = debugKdSupplier;
        this.debugIZoneSupplier = debugIZoneSupplier;
        this.debugPositionToleranceSupplier = debugPositionToleranceSupplier;
        this.debugVelocityToleranceSupplier = debugVelocityToleranceSupplier;
        this.debugIntegralMinRangeSupplier = debugIntegralMinRangeSupplier;
        this.debugIntegralMaxRangeSupplier = debugIntegralMaxRangeSupplier;
        this.debugKsSupplier = debugKsSupplier;
        this.debugKvSupplier = debugKvSupplier;
        this.debugKaSupplier = debugKaSupplier;

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
                    this::withVelocityTolerance
            );
            MMUtils.updateIfChanged(
                    debugVelocityToleranceSupplier,
                    pidController::getErrorDerivativeTolerance,
                    this::withAccelerationTolerance
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

            MMUtils.updateIfChanged(
                    debugKsSupplier,
                    feedforward::getKs,
                    feedforward::setKs
            );

            MMUtils.updateIfChanged(
                    debugKvSupplier,
                    feedforward::getKv,
                    feedforward::setKv
            );

            MMUtils.updateIfChanged(
                    debugKaSupplier,
                    feedforward::getKa,
                    feedforward::setKa
            );
        }
    }
}
