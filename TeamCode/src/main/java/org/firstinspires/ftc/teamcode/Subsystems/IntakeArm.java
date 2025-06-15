package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;

import Ori.Coval.Logging.AutoLog;

@Config
@AutoLog
public class IntakeArm extends ServoSubsystem {
    public static double intakeSample = 0.57;
    public static double prepareIntakeSample = 0.5;
    public static double specimenIntake = 0.2;
    public static double transferSample = 0.03;
    public static double init = 0.05;
    private static IntakeArm instance;


    public static synchronized IntakeArm getInstance() {
        if (instance == null) {
            instance = new IntakeArmAutoLogged();
        }
        return instance;
    }
    public IntakeArm() {
        super("IntakeArm");
        withServo(5,MMRobot.getInstance().controlHub, Direction.FORWARD,0.0);
        withServo(1, MMRobot.getInstance().controlHub, Direction.REVERSE,0.015);
    }
}
