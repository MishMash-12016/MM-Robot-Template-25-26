package org.firstinspires.ftc.teamcode.Robotcommand;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MMRobot;

public class ServoMoving {

    public static Command ServomoveOpen(){
        return MMRobot.getInstance().servoMoving.Openservo();
    }

    public static Command ServomoveClose(){
        return MMRobot.getInstance().servoMoving.Closeservo();
    }

    // Example of a more complex command sequence
    public static Command OpenThenWaitThenClose() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().servoMoving.Openservo(),
                new WaitCommand(1000),
                MMRobot.getInstance().servoMoving.Closeservo()
        );
    }
}
