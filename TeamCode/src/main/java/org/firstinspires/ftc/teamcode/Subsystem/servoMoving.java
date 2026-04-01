package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;

import Ori.Coval.Logging.AutoLog;

@Config
@AutoLog
public class servoMoving extends ServoSubsystem {

    public enum State {
        OPEN(0.95),
        CLOSE(0.00);

        public final double position;
        State(double position){
            this.position = position;
        }
    }

    public servoMoving(String subsystemName) {
        super(subsystemName);
        // Initialize the servo with the name "MoveServo"
        withServo("MoveServo", Direction.FORWARD, 0);
    }

    public Command Openservo(){
        return new InstantCommand(() -> setPosition(State.OPEN.position), this);
    }

    public Command Closeservo(){
        return new InstantCommand(() -> setPosition(State.CLOSE.position), this);
    }
}
