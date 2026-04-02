package org.firstinspires.ftc.teamcode;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.CentimetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMRobotInner;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.FeedForwards.FeedForwardType;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Position.PositionProfiledPidSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.servoMoving;

import edu.wpi.first.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public class MMRobot extends MMRobotInner {

    public servoMoving servoMoving;

    public MMRobot(){
        setControlHubName("Control Hub");
        setExpansionHubName("Expansion Hub");

        servoMoving = new servoMoving("Servo Moving");
    }

    @Override
    public void initAuto() {
        super.initAuto();
    }

    @Override
    public void initTele() {
        super.initTele();
    }
    
    @Override
    public void initDebug() {
        super.initDebug();
    }

}
