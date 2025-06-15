package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;

import Ori.Coval.Logging.AutoLog;

@Config
@AutoLog
public class ScoringClaw extends ServoSubsystem {
    public static double closePose = 0.48;
    public static double openPose = 0.95;
    private static ScoringClaw instance;
    public static synchronized ScoringClaw getInstance(){
        if (instance == null) {
            instance = new ScoringClawAutoLogged();
        }
        return instance;
    }

    public ScoringClaw() {
        super("ScoringClaw");
        withServo(0, MMRobot.getInstance().expansionHub, Direction.FORWARD,0.0);
    }



}
