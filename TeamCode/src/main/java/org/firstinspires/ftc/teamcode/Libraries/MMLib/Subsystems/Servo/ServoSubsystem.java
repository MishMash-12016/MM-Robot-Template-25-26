package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;

import java.util.ArrayList;
import java.util.Set;
import java.util.function.BooleanSupplier;

//TODO: decide if the user should bring a ready to use cuttle servo or the variables and we create the cuttle servo here
public class ServoSubsystem extends SubsystemBase {

    ArrayList<CuttleServo> servoList = new ArrayList<>();

    //TODO: decide if all the this needs to be here or not
    //on the one hand you must have all of the variables that are in here and the subsystem won't work without them
    //and i makes it impossible for the user to not have them
    //on the other hand, it is a bit annoying
    public ServoSubsystem(CuttleRevHub revHub, int servoPort, Direction servoDirection, Double offset) {
        CuttleServo servo = new CuttleServo(revHub, servoPort).setOffset(offset).setDirection(servoDirection);
        servoList.add(servo);
    }

    //TODO: decide if all the this needs to be here or not
    //on the one hand you must have all of the variables that are in here and the subsystem won't work without them
    //and i makes it impossible for the user to not have them
    //on the other hand, it is a bit annoying
    public ServoSubsystem(HardwareMap hardwareMap, String servoName, Direction servoDirection, Double offset) {
        CuttleServo servo = new CuttleServo(hardwareMap, servoName).setOffset(offset).setDirection(servoDirection);
        servoList.add(servo);
    }

    /**
     * moves the servo's to the target position
     * @param position the target position
     * @return instant command
     */
    public Command setPositionCommand(double position) {
        return new InstantCommand(() -> setPosition(position));
    }

    /**
     * the servo's will not immediately move to the target position it will take timeMS to complete the movement
     * @param targetPose the target position
     * @param movementDurationMS the time in milliseconds the servo should take to complete the movement
     * @return the command
     */
    public Command setPositionWithDuration(double targetPose, double movementDurationMS) {
        return new Command() {
            ElapsedTime currentTimeMS;
            double difference;
            double startPose;

            @Override
            public void initialize() {
                currentTimeMS = new ElapsedTime();
                currentTimeMS.reset();
                startPose = getPosition();
                difference = targetPose - startPose;

            }

            //slowly move the servo's to the target position
            @Override
            public void execute() {
                setPosition(startPose + (currentTimeMS.milliseconds() / movementDurationMS) * difference);
            }

            @Override
            public boolean isFinished() {
                return currentTimeMS.milliseconds() >= movementDurationMS;
            }

            @Override
            public void end(boolean interrupted) {
                setPosition(targetPose);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(ServoSubsystem.this);
            }
        };
    }

    public Command setPositionByButton(double defaultPosition,BooleanSupplier button, double position) {
        return new RunCommand(()->{
            if (button.getAsBoolean()) {
                setPosition(position);
            } else {
                setPosition(defaultPosition);
            }
        });
    }

    public Command setPositionByButton(double defaultPosition,BooleanSupplier button1, double position1, BooleanSupplier button2, double position2) {
        return new RunCommand(()->{
            if (button1.getAsBoolean()) {
                setPosition(position1);
            } else if(button2.getAsBoolean()) {
                setPosition(position2);
            } else {
                setPosition(defaultPosition);
            }
        });
    }

    public void setPosition(double position) {
        for(CuttleServo servo : servoList) {
            servo.setPosition(position);
        }
    }

    public double getPosition() {
        return servoList.get(0).getPosition();
    }

    //TODO: decide if the user should bring a ready to use cuttle servo or the variables and we create the cuttle servo here
    public ServoSubsystem withServo(CuttleServo servo) {
        servoList.add(servo);
        return this;
    }
}
