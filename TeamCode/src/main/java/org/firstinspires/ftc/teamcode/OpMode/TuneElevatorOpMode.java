package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

@TeleOp
@Config
public class TuneElevatorOpMode extends MMOpMode {

    //this is an example op mode to tune the elevator remember you can change the pid values using the ftc dashboard

    //public static value so it can be changed using the ftc dashboard
    public static double KS = 0.0;

    public static double targetPose = 0.0;

    public static double testPower = 0.0;

    public TuneElevatorOpMode() {
        super(OpModeType.NonCompetition.DEBUG);
    }

    @Override
    public void onInit() {
        GamepadEx gamepad1 = MMRobot.getInstance().gamepadEx1;

        //disable default command
        ElevatorSubsystem.getInstance().withSetDefaultCommand(ElevatorSubsystem.getInstance().setPowerRunCommand(0.0));

        //using "whenHeld" for safety so the command will stop when releasing the button
        gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld(ElevatorSubsystem.getInstance().tuneKSCommand(0.01,0.2));
        gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld(ElevatorSubsystem.getInstance().tuneKVCommand(0.1,KS));

        //turns the elevator pid on and off
        gamepad1.getGamepadButton(GamepadKeys.Button.CIRCLE).toggleWhenPressed(ElevatorSubsystem.getInstance().getToAndHoldSetPointCommand(()->targetPose));

        //set the test power for testing the ks and kv
        gamepad1.getGamepadButton(GamepadKeys.Button.SQUARE).whenHeld(ElevatorSubsystem.getInstance().setPowerRunCommand(testPower));
    }

    @Override
    public void onInitLoop() {}

    @Override
    public void onPlay() {}

    @Override
    public void onPlayLoop() {}

    @Override
    public void onEnd() {}
}
