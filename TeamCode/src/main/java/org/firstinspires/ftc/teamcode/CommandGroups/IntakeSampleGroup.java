package org.firstinspires.ftc.teamcode.CommandGroups;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeRotator;
import org.firstinspires.ftc.teamcode.Subsystems.LinearIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringClaw;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringElbow;

import java.util.function.BooleanSupplier;

public class IntakeSampleGroup {

    public static Command getPrepareIntakeSample() {

        int intakeArmTime = 300;

        BooleanSupplier isRotatorLeft = ()-> MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get();
        BooleanSupplier isRotatorRight = ()-> MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get();

        return new ParallelCommandGroup(
                //prepare scoring for transfer
                ScoringElbow.getInstance().setPositionCommand(ScoringElbow.scoreSamplePose),
                ScoringArm.getInstance().setPositionCommand(ScoringArm.prepareSampleTransferPose),
                ScoringClaw.getInstance().setPositionCommand(ScoringClaw.openPose),

                LinearIntake.getInstance().setPositionCommand(LinearIntake.open),

                //slowing down the intake arm a bit so it won't hit the sample
                IntakeArm.getInstance().setPositionOverTimeCommand(IntakeArm.intakeSample, intakeArmTime),
                new WaitCommand(intakeArmTime).andThen(IntakeClaw.getInstance().setPositionCommand(IntakeClaw.open)),

                IntakeRotator.getInstance().setPositionByButtonCommand(
                        IntakeRotator.defaultPose,
                        isRotatorLeft, IntakeRotator.leftAnglePose,
                        isRotatorRight, IntakeRotator.rightAnglePose)
        );
    }

    public static Command getIntakeSample() {
        int waitForIntakeArm = 200;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        ScoringElbow.getInstance().setPositionCommand(ScoringElbow.scoreSamplePose),
                        ScoringArm.getInstance().setPositionCommand(ScoringArm.prepareSampleTransferPose),
                        ScoringClaw.getInstance().setPositionCommand(ScoringClaw.openPose),

                        LinearIntake.getInstance().setPositionCommand(LinearIntake.open),
                        IntakeArm.getInstance().setPositionCommand(IntakeArm.intakeSample),
                        IntakeClaw.getInstance().setPositionCommand(IntakeClaw.open)
                ),

                new WaitCommand(waitForIntakeArm),
                IntakeClaw.getInstance().setPositionCommand(IntakeClaw.close),
                new WaitCommand(waitForIntakeArm),

                new ParallelCommandGroup(
                        IntakeArm.getInstance().setPositionCommand(IntakeArm.init),
                        IntakeRotator.getInstance().setPositionCommand(IntakeRotator.defaultPose),
                        LinearIntake.getInstance().setPositionCommand(LinearIntake.close)
                )
        );
    }
}
