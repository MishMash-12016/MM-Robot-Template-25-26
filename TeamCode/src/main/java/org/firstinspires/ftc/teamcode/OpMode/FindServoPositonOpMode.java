package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;

@TeleOp
@Config
public class FindServoPositonOpMode extends MMOpMode {

    ServoSubsystem servoSubsystem;

    public static double testPosition = 0.0;

    public FindServoPositonOpMode() {
        super(OpModeType.NonCompetition.DEBUG);
    }

    @Override
    public void onInit() {
        //change to which subsystem you want to use
        servoSubsystem = IntakeArm.getInstance();
    }

    @Override
    public void onInitLoop() {}

    @Override
    public void onPlay() {}

    @Override
    public void onPlayLoop() {
        servoSubsystem.setPosition(testPosition);
    }

    @Override
    public void onEnd() {}
}
