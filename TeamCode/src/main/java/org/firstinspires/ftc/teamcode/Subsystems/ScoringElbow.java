package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;

import Ori.Coval.Logging.AutoLog;

@Config
@AutoLog
public class ScoringElbow extends ServoSubsystem  {
    public static double prepareSampleScorePose = 0.42;
    public static double scoreSamplePose = 0.78;
    public static double initPose = 0.32;
    public static double transferSamplePose = 0.1;
    public static double prepareSampleTransferPose = 0.05;
    public static double scoreSpecimenPose = 0.74;
    public static double afterScoreSpecimenPose = 0.64;
    public static double intakeFromFrontPose = 0.4;
    public static double scoringScoreFromFrontPose = 0.53;
    public static double afterScoreFromFront = 0.35;
    public static double park = 0.38;

    private static ScoringElbow instance;

    public static synchronized ScoringElbow getInstance() {
        if (instance == null) {
            instance = new ScoringElbowAutoLogged();
        }
        return instance;
    }
    public ScoringElbow() {
        super("ScoringElbow");
        withServo("ScoringElbowServo", Direction.FORWARD, 0.0);
    }

}
