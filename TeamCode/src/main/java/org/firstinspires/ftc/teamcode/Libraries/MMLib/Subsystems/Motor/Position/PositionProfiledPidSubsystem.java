package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Position;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.tuning.FFKsSysid;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.tuning.FFKvSysid;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Base.PidBaseSubsystem;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMUtils;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.MMRobot;
import java.util.Set;
import java.util.function.DoubleSupplier;

import Ori.Coval.Logging.Logger.KoalaLog;

public class PositionProfiledPidSubsystem extends PidBaseSubsystem {

    public PositionProfiledPidSubsystem(String subsystemName) {
        super(subsystemName);
        this.pidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    }

    /**
     * Creates a Command that keeps the mechanism in place using Profiled  control.
     *
     * @return a Command requiring this subsystem
     */
    @Override
    public Command getToAndHoldSetPointCommand(DoubleSupplier setPoint) {
        return new Command() {
            @Override
            public void initialize() {
                // clear previous errors/integral
                ((ProfiledPIDController) pidController).reset(getPose(), getVelocity());
                pidController.setSetpoint(setPoint.getAsDouble());

                KoalaLog.log(subsystemName + "/pid setpoint", setPoint.getAsDouble(), true);
            }

            @Override
            public void execute() {
                pidController.setSetpoint(setPoint.getAsDouble());

                KoalaLog.log(subsystemName + "/pid setpoint", setPoint.getAsDouble(), true);
                
                double pidOutput = KoalaLog.log(subsystemName + "/pid output", pidController.calculate(getPose()), true);
                double feedforwardOutput = 0;


                if (feedforward != null) {
                    feedforwardOutput = feedforward.calculate(((ProfiledPIDController) pidController).getSetpointState().velocity);
                }
                KoalaLog.log(subsystemName + "/pid feedforward", feedforwardOutput, true);

                setPower(pidOutput + feedforwardOutput);// apply computed power
            }

            @Override
            public Set<Subsystem> getRequirements() {
                // Declare that this command requires the enclosing subsystem instance
                return Set.of(PositionProfiledPidSubsystem.this);
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
    public PositionProfiledPidSubsystem withFeedforward(double ks, double kv, double ka) {
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        return this;
    }

    /**
     * Updates feedforward gain.
     *
     * @param ks static gain
     */
    public PositionProfiledPidSubsystem withKs(double ks) {
        feedforward.setKs(ks);
        return this;
    }

    /**
     * Updates feedforward gain.
     *
     * @param kv velocity gain
     */
    public PositionProfiledPidSubsystem withKv(double kv) {
        feedforward.setKv(kv);
        return this;
    }

    /**
     * Updates feedforward gain.
     *
     * @param ka acceleration gain
     */
    public PositionProfiledPidSubsystem withKa(double ka) {
        feedforward.setKa(ka);
        return this;
    }

    /**
     * updates the constraints values
     *
     * @param maxVelocity     the maximum velocity
     * @param maxAcceleration the maximum acceleration
     * @return this subsystem for chaining
     */
    public PositionProfiledPidSubsystem withConstraints(double maxVelocity, double maxAcceleration) {
        ((ProfiledPIDController) pidController).setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        return this;
    }

    /**
     * updates the constraint values
     *
     * @param maxVelocity the maximum velocity
     * @return this subsystem for chaining
     */
    public PositionProfiledPidSubsystem withMaxVelocityConstraint(double maxVelocity) {
        ((ProfiledPIDController) pidController)
                .setConstraints(new TrapezoidProfile.Constraints(maxVelocity, ((ProfiledPIDController) pidController).getConstraints().maxAcceleration));
        return this;
    }

    /**
     * updates the constraint values
     *
     * @param maxAcceleration the maximum acceleration
     * @return this subsystem for chaining
     */
    public PositionProfiledPidSubsystem withMaxAccelerationConstraint(double maxAcceleration) {
        ((ProfiledPIDController) pidController)
                .setConstraints(new TrapezoidProfile.Constraints(((ProfiledPIDController) pidController).getConstraints().maxAcceleration, maxAcceleration));
        return this;
    }

    /**
     * Sets the acceptable position error tolerance for the PID setpoint.
     *
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public PositionProfiledPidSubsystem withPositionTolerance(double tolerance) {
        pidController.setTolerance(tolerance);
        return this;
    }

    /**
     * Sets the acceptable velocity error tolerance for the PID setpoint.
     *
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public PositionProfiledPidSubsystem withVelocityTolerance(double tolerance) {
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
    private DoubleSupplier debugMaxVelocitySupplier;
    private DoubleSupplier debugMaxAccelerationSupplier;

    /**
     * add suppliers that when changed will auto update the pid values.
     * any value you don't need just put null
     *
     * @param debugKpSupplier                  Kp
     * @param debugKiSupplier                  Kd
     * @param debugKdSupplier                  Ki
     * @param debugIZoneSupplier               iZone
     * @param debugPositionToleranceSupplier position tolerance
     * @param debugVelocityToleranceSupplier velocity tolerance
     * @param debugIntegralMinRangeSupplier    integral min range
     * @param debugIntegralMaxRangeSupplier    integral max range
     * @param debugKsSupplier                  static gain
     * @param debugKvSupplier                  velocity gain
     * @param debugKaSupplier                  acceleration gain
     * @param debugMaxVelocitySupplier         max velocity
     * @param debugMaxAccelerationSupplier     max acceleration
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
                                                  DoubleSupplier debugKaSupplier,
                                                  DoubleSupplier debugMaxVelocitySupplier,
                                                  DoubleSupplier debugMaxAccelerationSupplier) {

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
        this.debugMaxVelocitySupplier = debugMaxVelocitySupplier;
        this.debugMaxAccelerationSupplier = debugMaxAccelerationSupplier;

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
                    pidController::getErrorDerivativeTolerance,
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

            MMUtils.updateIfChanged(
                    debugMaxVelocitySupplier,
                    () -> ((ProfiledPIDController) pidController).getConstraints().getMaxVelocity(),
                    ((ProfiledPIDController) pidController)::setMaxVelocity
            );

            MMUtils.updateIfChanged(
                    debugMaxAccelerationSupplier,
                    () -> ((ProfiledPIDController) pidController).getConstraints().getMaxAcceleration(),
                    ((ProfiledPIDController) pidController)::setMaxAcceleration
            );
        }
    }
}
