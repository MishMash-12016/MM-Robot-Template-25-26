package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.PIDController;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.SimpleMotorFeedforward;

import java.util.ArrayList;
import java.util.Set;

/**
 * MotorVelocityPidSubsystem provides PID-controlled velocity for a motor-driven mechanism.
 *
 * <p>This subsystem uses a CuttleEncoder to measure velocity, a list of CuttleMotors to apply power,
 * and a PIDController to compute the required power to reach a target velocity setpoint.
 * </p>
 */
//TODO: decide if the user should bring a ready to use cuttle motor or the variables and we create the cuttle motor here
public class MotorVelocityPidSubsystem extends SubsystemBase {

    private final ArrayList<CuttleMotor> motorList = new ArrayList<>();
    private final CuttleEncoder encoder;
    private final PIDController pidController;
    private final SimpleMotorFeedforward feedforward;

    /**
     * Constructs a MotorVelocityPidSubsystem with given PID gains and encoder.
     *
     * @param kp                 proportional gain
     * @param ki                 integral gain
     * @param kd                 derivative gain
     * @param encoderPort        Port number the encoder is connected to.
     * @param encoderCPR         Counts per revolution (CPR) of the encoder.
     * @param encoderDirection   Direction configuration of the encoder.
     * @param motorPort          Port number the motor is connected to.
     * @param motorDirection     Direction config of the motor.
     * @param withDefaultCommand whether to set the default hold command.
     * @param revHub             the REV Hub instance.
     */
    //TODO: decide if all the this needs to be here or not
    //on the one hand you must have all of the variables that are in here and the subsystem won't work without them
    //and i makes it impossible for the user to not have them
    //on the other hand, it is a bit annoying
    public MotorVelocityPidSubsystem(double kp, double ki, double kd, double kS, double kV,
                                     int encoderPort, double encoderCPR, Direction encoderDirection,
                                     int motorPort, Direction motorDirection,
                                     boolean withDefaultCommand,
                                     CuttleRevHub revHub) {

        this.pidController = new PIDController(kp, ki, kd);
        this.feedforward = new SimpleMotorFeedforward(kS, kV);

        this.encoder = new CuttleEncoder(revHub, encoderPort, encoderCPR)
                .setDirection(encoderDirection);
        motorList.add(new CuttleMotor(revHub, motorPort).setDirection(motorDirection));

        if (withDefaultCommand) {
            setDefaultCommand(holdVelocityCommand());
        }
    }

    /**
     * Convenience constructor with default hold command enabled.
     */
    public MotorVelocityPidSubsystem(double kp, double ki, double kd, double kS, double kV,
                                     int encoderPort, double encoderCPR, Direction encoderDirection,
                                     int motorPort, Direction motorDirection,
                                     CuttleRevHub revHub) {

        this(kp, ki, kd, kS, kV, encoderPort, encoderCPR, encoderDirection, motorPort, motorDirection, true, revHub);
    }

    /**
     * Returns the current velocity measured by the encoder (in units per second).
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Sets raw power to all motors.
     */
    public void setPower(double power) {
        for (CuttleMotor m : motorList) {
            m.setPower(power);
        }
    }

    /**
     * Command to set a target velocity.
     */
    public Command setVelocityCommand(double targetVelocity) {
        return new Command() {
            @Override
            public void initialize() {
                pidController.reset();
                pidController.setSetpoint(targetVelocity);
            }

            @Override
            public void execute() {
                double output = pidController.calculate(getVelocity());
                double feedforwardOutput = feedforward.calculate(getVelocity());
                setPower(output + feedforwardOutput);
            }

            @Override
            public boolean isFinished() {
                return pidController.atSetpoint();
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(MotorVelocityPidSubsystem.this);
            }
        };
    }

    /**
     * Default command that holds current velocity.
     */
    public Command holdVelocityCommand() {
        return new Command() {
            @Override
            public void initialize() {
                pidController.reset();
                pidController.setSetpoint(getVelocity());
            }

            @Override
            public void execute() {
                double output = pidController.calculate(getVelocity());
                setPower(output);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(MotorVelocityPidSubsystem.this);
            }
        };
    }

    /**
     * Adds another motor to be controlled.
     */
    public MotorVelocityPidSubsystem withMotor(CuttleMotor motor) {
        motorList.add(motor);
        return this;
    }

    /**
     * Updates PID gains.
     */
    public MotorVelocityPidSubsystem withPid(double kp, double ki, double kd) {
        pidController.setPID(kp, ki, kd);
        return this;
    }

    /**
     * Updates feedforward gains.
     *
     * @param kS feedforward gain
     * @param kV feedforward gain
     * @return this subsystem for chaining
     */
    public MotorVelocityPidSubsystem withFeedforward(double kS, double kV) {
        feedforward.setKs(kS);
        feedforward.setKv(kV);
        return this;
    }

    /**
     * Sets tolerance for velocity error.
     */
    public MotorVelocityPidSubsystem withVelocityTolerance(double tolerance) {
        pidController.setTolerance(tolerance, pidController.getErrorDerivativeTolerance());
        return this;
    }

    public MotorVelocityPidSubsystem withAccelerationTolerance(double tolerance) {
        pidController.setTolerance(pidController.getErrorTolerance(), tolerance);
        return this;
    }

    /**
     * Sets I-zone for integral control.
     */
    public MotorVelocityPidSubsystem withIZone(double iZone) {
        pidController.setIZone(iZone);
        return this;
    }

    /**
     * Sets bounds on integral accumulator.
     */
    public MotorVelocityPidSubsystem withIntegralRange(double min, double max) {
        pidController.setIntegratorRange(min, max);
        return this;
    }
}
