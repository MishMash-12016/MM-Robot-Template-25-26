package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;

import Ori.Coval.Logging.AutoLog;

@Config
@AutoLog
public class IntakeClaw extends ServoSubsystem {
    public static double open = 0.9;
    public static double close = 0.6;
    private static IntakeClaw instance;

    public static synchronized IntakeClaw getInstance() {
        if (instance == null) {
            instance = new IntakeClawAutoLogged();
        }
        return instance;
    }
    public IntakeClaw(){
        super("IntakeClaw");
        withServo(0, MMRobot.getInstance().expansionHub, Direction.FORWARD,0.0);
    }
}
