package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleDigital;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.PIDController;
import org.firstinspires.ftc.teamcode.MMSystems;

import java.util.ArrayList;
import java.util.Set;


/**
 * MotorPositionPidSubsystem provides PID-controlled positioning for a motor-driven mechanism.
 * <p>
 * This subsystem uses a CuttleEncoder to measure position, a list of CuttleMotors to apply power,
 * and a PIDController to compute the required power to reach a target setpoint.
 * </p>
 * Usage example:
 * <pre>
 *   CuttleEncoder encoder = new CuttleEncoder(...);
 *   MotorPositionPidSubsystem arm = new MotorPositionPidSubsystem(1.0, 0.0, 0.1, encoder)
 *       .withMotor(new CuttleMotor(...))
 *       .withTolerance(0.05)
 *       .withRatio(13);
 *   Command moveArm = arm.moveToPoseCommand(100);
 * </pre>
 */

//TODO: decide if the user should bring a ready to use cuttle motor or the variables and we create the cuttle motor here
public class MotorPositionPidSubsystem extends SubsystemBase {

    // List of motors driven by this subsystem
    private final ArrayList<CuttleMotor> motorList = new ArrayList<>();
    // Encoder that measures current position (ticks converted via ratio)
    private final CuttleEncoder encoder;
    // PID controller for calculating output power
    private final PIDController pidController;

    /**
     * Constructs a MotorPositionPidSubsystem with given PID gains and encoder.
     * @param kp proportional gain
     * @param ki integral gain
     * @param kd derivative gain
     * @param encoderPort      Port number the encoder is connected to.
     * @param encoderCPR       Counts per revolution (CPR) of the encoder.
     * @param encoderDirection Direction configuration of the encoder (e.g. forward or reverse).
     * @param motorPort        Port number the motor is connected to.
     * @param motorDirection   Direction configuration of the motor (e.g. forward or
     * @param withDefaultCommand false if you don't want the default command (value default is true)
     */
    //TODO: decide if all the this needs to be here or not
    //on the one hand you must have all of the variables that are in here and the subsystem won't work without them
    //and i makes it impossible for the user to not have them
    //on the other hand, it is a bit annoying
    public MotorPositionPidSubsystem(double kp, double ki, double kd,
                                     int encoderPort, double encoderCPR, Direction encoderDirection,
                                     int motorPort, Direction motorDirection,
                                     boolean withDefaultCommand,
                                     CuttleRevHub revHub) {
        this.pidController = new PIDController(kp, ki, kd);
        this.encoder = new CuttleEncoder(revHub, encoderPort, encoderCPR)
                .setDirection(encoderDirection);

        motorList.add(new CuttleMotor(revHub, motorPort).setDirection(motorDirection));

        if (withDefaultCommand){
            this.setDefaultCommand(stayAtPoseCommand());
        }
    }


    /**
     * Constructs a MotorPositionPidSubsystem with given PID gains and encoder.
     * @param kp proportional gain
     * @param ki integral gain
     * @param kd derivative gain
     * @param encoderPort      Port number the encoder is connected to.
     * @param encoderCPR       Counts per revolution (CPR) of the encoder.
     * @param encoderDirection Direction configuration of the encoder (e.g. forward or reverse).
     * @param motorPort        Port number the motor is connected to.
     * @param motorDirection   Direction configuration of the motor (e.g. forward or
     */
    public MotorPositionPidSubsystem(double kp, double ki, double kd,
                                     int encoderPort, double encoderCPR, Direction encoderDirection,
                                     int motorPort, Direction motorDirection,
                                     CuttleRevHub revHub) {
        this(kp, ki, kd, encoderPort, encoderCPR, encoderDirection,
                motorPort, motorDirection, true, revHub);
    }

    /**
     * Creates a Command that moves the mechanism to the specified setpoint using PID control.
     * @param setPoint target position (in encoder units, adjusted by ratio)
     * @return a Command requiring this subsystem
     */
    public Command moveToPoseCommand(double setPoint) {
        return new Command() {
            @Override
            public void initialize() {
                pidController.reset();             // clear previous errors/integral
                pidController.setSetpoint(setPoint);
            }

            @Override
            public void execute() {
                double output = pidController.calculate(getPose());
                setPower(output);                 // apply computed power
            }

            @Override
            public boolean isFinished() {
                return pidController.atSetpoint(); // check if within tolerance
            }

            @Override
            public Set<Subsystem> getRequirements() {
                // Declare that this command requires the enclosing subsystem instance
                return Set.of(MotorPositionPidSubsystem.this);
            }
        };
    }

    /**
     * Creates a Command that keeps the mechanism in place using PID control.
     *
     * @return a Command requiring this subsystem
     */
    public Command stayAtPoseCommand() {
        return new Command() {
            @Override
            public void initialize() {
                pidController.reset();             // clear previous errors/integral
                pidController.setSetpoint(getPose());
            }

            @Override
            public void execute() {
                double output = pidController.calculate(getPose());
                setPower(output);                 // apply computed power
            }

            @Override
            public Set<Subsystem> getRequirements() {
                // Declare that this command requires the enclosing subsystem instance
                return Set.of(MotorPositionPidSubsystem.this);
            }
        };
    }

    /**
     * Returns the current position (pose) provided by the encoder.
     * @return current pose in encoder units (divided by ratio)
     */
    public double getPose() {
        return encoder.getPose();
    }

    /**
     * Creates a Command that sets the motor power directly.
     * <p>When this command finishes, it resets the PID setpoint to the current pose,
     * holding position at that point.</p>
     * @param power motor power (-1.0 to 1.0)
     * @return a RunCommand requiring this subsystem
     */
    public Command setPowerCommand(double power) {
        return new RunCommand(() -> setPower(power), this)
                .whenFinished(() -> pidController.setSetpoint(getPose()));
    }

    /**
     * @apiNote !NOTICE THIS IS NOT A COMMAND AND WILL NOT STOP THE DEFAULT COMMAND
     * <p>Sets the raw power to all motors. Use with caution if a default command
     * is installed, as this method does not manage command requirements.</p>
     * @param power motor power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        for (CuttleMotor motor : motorList) {
            motor.setPower(power);
        }
    }

    /**
     *
     * @param setpoint the setpoint for the pid to aim for
     */
    public void setSetpoint(double setpoint){
        pidController.setSetpoint(setpoint);
    }

    /**
     * Adds a motor to the subsystem.
     * @param motor a CuttleMotor instance
     * @return this subsystem for chaining
     */
    public MotorPositionPidSubsystem withMotor(CuttleMotor motor) {
        motorList.add(motor);
        return this;
    }

    /**
     * Configures a zero-position limit switch that resets encoder when activated.
     * @param zeroSwitch digital switch input
     * @param zeroPose encoder value to set when switch is pressed
     * @return this subsystem for chaining
     */
    public MotorPositionPidSubsystem withZeroSwitch(CuttleDigital zeroSwitch, double zeroPose) {
        new Trigger(zeroSwitch::getState)
                .whenActive(() -> encoder.setPose(zeroPose));
        return this;
    }

    /**
     * Updates PID gains.
     * @param kp proportional gain
     * @param ki integral gain
     * @param kd derivative gain
     * @return this subsystem for chaining
     */
    public MotorPositionPidSubsystem withPid(double kp, double ki, double kd) {
        pidController.setPID(kp, ki, kd);
        return this;
    }

    /**
     * Sets the acceptable position error tolerance for the PID setpoint.
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public MotorPositionPidSubsystem withPositionTolerance(double tolerance) {
        pidController.setTolerance(tolerance);
        return this;
    }

    /**
     * Sets the acceptable velocity error tolerance for the PID setpoint.
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public MotorPositionPidSubsystem withVelocityTolerance(double tolerance) {
        pidController.setTolerance(pidController.getErrorTolerance(),tolerance);
        return this;
    }

    /**
     * Defines the Integral Zone (I-Zone) for the PID controller.
     * @param iZone range around setpoint where integral is active
     * @return this subsystem for chaining
     */
    public MotorPositionPidSubsystem withIZone(double iZone) {
        pidController.setIZone(iZone);
        return this;
    }

    /**
     * Restricts the integral accumulator within given bounds.
     * @param minIntegralRange minimum accumulator value
     * @param maxIntegralRange maximum accumulator value
     * @return this subsystem for chaining
     */
    public MotorPositionPidSubsystem withIntegralRange(double minIntegralRange, double maxIntegralRange) {
        pidController.setIntegratorRange(minIntegralRange, maxIntegralRange);
        return this;
    }
}
