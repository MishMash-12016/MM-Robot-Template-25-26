package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;

import Ori.Coval.Logging.AutoLog;

@Config
@AutoLog
public class IntakeRotator extends ServoSubsystem {
    public static double defaultPose = 0.33;
    public static double initPose = 0.98;
    public static double rightAnglePose = 0.57;
    public static double leftAnglePose = 0.11;
    private static IntakeRotator instance;

    public static synchronized IntakeRotator getInstance() {
        if (instance == null) {
            instance = new IntakeRotatorAutoLogged();
        }
        return instance;
    }
    public IntakeRotator(){
        super("IntakeRotator");
        withServo(3, MMRobot.getInstance().controlHub, Direction.FORWARD,0.0);
    }
}
