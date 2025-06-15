package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;

import Ori.Coval.Logging.AutoLog;


@Config
@AutoLog
public class ScoringArm extends ServoSubsystem {

    public static double initPose = 0.52;
    public static double sampleTransferPose = 0.54;
    public static double prepareSampleTransferPose = 0.45;
    public static double scorePose = 0.19;
    public static double intakeFromFrontPose = 0.545;
    public static double scoreFromFrontSpecimenPose = 0.42;
    public static double afterScoreFromFrontSpecimenPose = 0.3;
    public static double scoreSample = 0.24;
    public static double park = 0.39;


    // Singleton instance
    private static ScoringArm instance;

    /**
     * Get the singleton instance of ElevatorSubsystem.
     */
    public static synchronized ScoringArm getInstance() {
        if (instance == null) {
            instance = new ScoringArmAutoLogged();
        }
        return instance;
    }

    /**
     * Creates a ServoSubsystem
     *
     */
    public ScoringArm() {
        super("ScoringArm");
        withServo("L outake arm ", Direction.FORWARD,0.015);
        withServo("R outake arm ", Direction.REVERSE,0.0);
    }
}
