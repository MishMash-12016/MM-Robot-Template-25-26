package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.Robot;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;

public class MMRobot extends Robot {

    public MMOpMode currentOpMode;

    /**
     * the robot instance
     */
    private static MMRobot instance;

    /**
     * the get method for the singleton
     * @return the robot instance
     */
    public static synchronized MMRobot getInstance() {
        if (instance == null) {
            instance = new MMRobot();
        }
        return instance;
    }

    public synchronized void resetRobot(){
        instance = null;
    }

    /**
     * this initializes your subsystems.
     * <p>
     * if experimenting, then this does nothing.
     * @param type the {@link OpModeType} chosen
     */
    public void initializeSystems(OpModeType type) {
        if(type == OpModeType.Competition.TELEOP) {
            initTele();
        } else if (type == OpModeType.Competition.AUTO) {
            initAuto();
        } else if(type == OpModeType.NonCompetition.DEBUG) {
            initDebug();
        }
    }

    /**
     * initialize subsystems required for teleop
     */
    private void initTele() {
    }

    /**
     * initialize subsystems required for auto
     */
    private void initAuto() {
        //TODO a day before comp
    }

    /**
     * initialize subsystems required for debug mode
     */
    private void initDebug() {
        //TODO when mechanics tells u "hey can u make this servo move in weird ahh ways at 0.000001 speed"
    }
}
