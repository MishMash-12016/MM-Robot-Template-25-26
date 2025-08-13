package org.firstinspires.ftc.teamcode;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.CentimetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMRobotInner;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.FeedForwards.FeedForwardType;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Position.PositionProfiledPidSubsystem;

import edu.wpi.first.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;

public class MMRobot extends MMRobotInner {

    public MMRobot(){
        setControlHubName("Control Hub");
        setExpansionHubName("Expansion Hub");
    }

    @Override
    public void initAuto() {
        super.initAuto();
    }

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance m_position = Centimeters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity = CentimetersPerSecond.mutable(0);

    @Override
    public void initTele() {
        super.initTele();

        PositionProfiledPidSubsystem pidSubsystem = (PositionProfiledPidSubsystem) new PositionProfiledPidSubsystem("test")
                .withEncoder(controlHub, 0,15, Direction.FORWARD)
                .withPid(0,0,0)
                .withFeedforward(FeedForwardType.ELEVATOR,0,0,0,0)
                .withConstraints(10,10)
                .withMotor(controlHub, 0, Direction.FORWARD);

        SysIdRoutine m_sysIdRoutine =
                new SysIdRoutine(
                        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                // Tell SysId how to plumb the driving voltage to the motor(s).
                                pidSubsystem::setPower,
                                // Tell SysId how to record a frame of data for each motor on the mechanism being
                                // characterized.
                                log -> {
                                    // Record a frame for the shooter motor.
                                    log.motor("shooter-wheel")
                                            .voltage(
                                                    m_appliedVoltage.mut_replace(
                                                            pidSubsystem.getPower(), Volts))
                                            .linearPosition(m_position.mut_replace(pidSubsystem.getPose(), Centimeters))
                                            .linearVelocity(
                                                    m_velocity.mut_replace(pidSubsystem.getVelocity(), CentimetersPerSecond));
                                },
                                // Tell SysId to make generated commands require this subsystem, suffix test state in
                                // WPILog with this subsystem's name ("shooter")
                                pidSubsystem));


    }

    @Override
    public void initDebug() {
        super.initDebug();
    }
}
