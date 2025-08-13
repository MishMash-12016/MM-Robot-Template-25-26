package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Base;

import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.ProfiledPIDController;
import com.seattlesolvers.solverslib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleDigital;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.FeedForwards.ArmFeedforward;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.FeedForwards.ElevatorFeedforward;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.FeedForwards.FeedForwardType;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.FeedForwards.SimpleMotorFeedforward;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMUtils;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import Ori.Coval.Logging.AutoLogOutput;


public class ProfiledPidBase extends MotorOrCrServoSubsystem {
    private double positionTolarence = 0;
    private double maxAcceleration = 0;
    private double maxVelocity = 0;

    // Encoder that measures current position and velocity (ticks converted via ratio)
    private CuttleEncoder encoder;
    public ProfiledPIDController profiledPIDController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0,0));
    public SimpleMotorFeedforward feedforward;//TODO:  make other feedforwards usable

    //base
    public ProfiledPidBase(String subsystemName) {
        super(subsystemName);
    }

    /**
     * Returns the current position (pose) provided by the encoder.
     *
     * @return current pose in encoder units (divided by ratio)
     */
    @AutoLogOutput
    public double getPose() {
        return encoder.getPose();
    }

    @AutoLogOutput
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @AutoLogOutput
    public boolean getAtGoal(){
        return profiledPIDController.atGoal();
    }

    @AutoLogOutput
    public double getPositonError(){
        return profiledPIDController.getPositionError();
    }

    @AutoLogOutput
    public double getVelocityError(){
        return profiledPIDController.getVelocityError();
    }

    @AutoLogOutput
    public double getSetPoint(){
        return profiledPIDController.getGoal().position;
    }

    public void setPose(double pose) {
        encoder.setPose(pose);
    }

    public ProfiledPidBase withEncoder(CuttleRevHub revHub, int encoderPort, double cpr, Direction direction) {
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
    public ProfiledPidBase withZeroSwitch(CuttleDigital zeroSwitch, double zeroPose) {
        new Trigger(zeroSwitch::getState)
                .whenActive(() -> encoder.setPose(zeroPose));
        return this;
    }

    public ProfiledPidBase withZeroSwitch(CuttleDigital zeroSwitch) {
        withZeroSwitch(zeroSwitch, 0);
        return this;
    }

    /**
     * Configures a zero-position limit switch that resets encoder when activated.
     *
     * @param zeroSupplier a supplier of when to zero the system
     * @param zeroPose     encoder value to set when switch is pressed
     * @return this subsystem for chaining
     */
    public ProfiledPidBase withZeroSupplier(BooleanSupplier zeroSupplier, double zeroPose) {
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
    public ProfiledPidBase withPid(double kp, double ki, double kd) {
        profiledPIDController.setPID(kp, ki, kd);
        return this;
    }

    /**
     * Updates feedforward gains.
     *
     * @param ks static gain
     * @param Kg the gravity gain leave empty for simple feedforward
     * @param kv velocity gain
     * @param ka acceleration gain
     */
    public ProfiledPidBase withFeedforward(FeedForwardType feedForwardType, double ks, double Kg, double kv, double ka) {
        if(feedForwardType == FeedForwardType.SIMPLE){
            feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        } else if (feedForwardType == FeedForwardType.ELEVATOR) {
            feedforward = new ElevatorFeedforward(ks, Kg, kv, ka);
        }else if (feedForwardType == FeedForwardType.ARM) {
            feedforward = new ArmFeedforward(ks, Kg, kv, ka);
        }
        return this;
    }

    /**
     * Updates feedforward gain.
     *
     * @param ks static gain
     */
    public ProfiledPidBase withKs(double ks) {
        feedforward.setKs(ks);
        return this;
    }

    /**
     * Updates feedforward gain.
     *
     * @param kv velocity gain
     */
    public ProfiledPidBase withKv(double kv) {
        feedforward.setKv(kv);
        return this;
    }

    /**
     * Updates feedforward gain.
     *
     * @param ka acceleration gain
     */
    public ProfiledPidBase withKa(double ka) {
        feedforward.setKa(ka);
        return this;
    }

    /**
     * Updates feedforward gain.
     *
     * @param kg gravity gain
     */
    public ProfiledPidBase withKg(double kg) {
        feedforward.setKg(kg);
        return this;
    }


    /**
     * updates the constraints values
     *
     * @param maxVelocity     the maximum velocity
     * @param maxAcceleration the maximum acceleration
     * @return this subsystem for chaining
     */
    public ProfiledPidBase withConstraints(double maxVelocity, double maxAcceleration) {
        profiledPIDController.setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        return this;
    }

    /**
     * updates the constraint values
     *
     * @param maxVelocity the maximum velocity
     * @return this subsystem for chaining
     */
    public ProfiledPidBase withMaxVelocityConstraint(double maxVelocity) {
        profiledPIDController
                .setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        this.maxVelocity = maxVelocity;
        return this;
    }

    /**
     * updates the constraint values
     *
     * @param maxAcceleration the maximum acceleration
     * @return this subsystem for chaining
     */
    public ProfiledPidBase withMaxAccelerationConstraint(double maxAcceleration) {
        profiledPIDController
                .setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        this.maxAcceleration = maxAcceleration;
        return this;
    }

    /**
     * Sets the acceptable position error tolerance for the PID setpoint.
     *
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public ProfiledPidBase withTolerance(double tolerance) {
        profiledPIDController.setTolerance(tolerance, Double.POSITIVE_INFINITY);
        positionTolarence = tolerance;
        return this;
    }

    @Override
    public void resetHub() {
        super.resetHub();
        double pose = getPose();
        if(encoder.hub.getHubName().equals(MMRobot.getInstance().controlHub.getHubName())){
            encoder.hub = MMRobot.getInstance().controlHub;
        }
        else {
            encoder.hub = MMRobot.getInstance().expansionHub;
        }
        setPose(pose);
    }


    private DoubleSupplier debugKpSupplier;
    private DoubleSupplier debugKiSupplier;
    private DoubleSupplier debugKdSupplier;
    private DoubleSupplier debugPositionToleranceSupplier;
    private DoubleSupplier debugKsSupplier;
    private DoubleSupplier debugKgSupplier;
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
     * @param debugPositionToleranceSupplier position tolerance
     * @param debugKsSupplier                  static gain
     * @param debugKgSupplier                  gravity gain
     * @param debugKvSupplier                  velocity gain
     * @param debugKaSupplier                  acceleration gain
     * @param debugMaxVelocitySupplier         max velocity
     * @param debugMaxAccelerationSupplier     max acceleration
     * @implNote !NOTICE THIS ONLY WORKS IF IN DEBUG MODE
     */
    public ProfiledPidBase withDebugPidSuppliers(DoubleSupplier debugKpSupplier,
                                                 DoubleSupplier debugKiSupplier,
                                                 DoubleSupplier debugKdSupplier,
                                                 DoubleSupplier debugPositionToleranceSupplier,
                                                 DoubleSupplier debugKsSupplier,
                                                 DoubleSupplier debugKgSupplier,
                                                 DoubleSupplier debugKvSupplier,
                                                 DoubleSupplier debugKaSupplier,
                                                 DoubleSupplier debugMaxVelocitySupplier,
                                                 DoubleSupplier debugMaxAccelerationSupplier) {

        this.debugKpSupplier = debugKpSupplier;
        this.debugKiSupplier = debugKiSupplier;
        this.debugKdSupplier = debugKdSupplier;
        this.debugPositionToleranceSupplier = debugPositionToleranceSupplier;
        this.debugKsSupplier = debugKsSupplier;
        this.debugKgSupplier = debugKgSupplier;
        this.debugKvSupplier = debugKvSupplier;
        this.debugKaSupplier = debugKaSupplier;
        this.debugMaxVelocitySupplier = debugMaxVelocitySupplier;
        this.debugMaxAccelerationSupplier = debugMaxAccelerationSupplier;

        return this;
    }


    @Override
    public void periodic() {
        super.periodic();
        if (MMRobot.getInstance().currentOpMode != null &&
                MMRobot.getInstance().currentOpMode.opModeType == OpModeType.NonCompetition.DEBUG) {

            MMUtils.updateIfChanged(
                    debugKpSupplier,
                    profiledPIDController::getP,
                    profiledPIDController::setP
            );
            MMUtils.updateIfChanged(
                    debugKiSupplier,
                    profiledPIDController::getI,
                    profiledPIDController::setI
            );
            MMUtils.updateIfChanged(
                    debugKdSupplier,
                    profiledPIDController::getD,
                    profiledPIDController::setD
            );
            MMUtils.updateIfChanged(
                    debugPositionToleranceSupplier,
                    ()->positionTolarence,
                    this::withTolerance
            );
            MMUtils.updateIfChanged(
                    debugKsSupplier,
                    feedforward::getKs,
                    feedforward::setKs
            );

            MMUtils.updateIfChanged(
                    debugKsSupplier,
                    feedforward::getKg,
                    feedforward::setKg
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
                    () -> maxVelocity,
                    this::withMaxVelocityConstraint
            );

            MMUtils.updateIfChanged(
                    debugMaxAccelerationSupplier,
                    () -> maxAcceleration,
                    this::withMaxAccelerationConstraint
            );
        }
    }
}
