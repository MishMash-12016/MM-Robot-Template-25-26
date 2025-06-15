package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommandGroups.IntakeSampleGroup;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMDrivetrain;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;

@TeleOp
public class TeleopOpMode extends MMOpMode {

    public TeleopOpMode() {
        super(OpModeType.Competition.TELEOP);
    }

    @Override
    public void onInit() {
        GamepadEx gamepad1 = MMRobot.getInstance().gamepadEx1;

        //turns on the mecanum drive
        MMDrivetrain.getInstance().enableTeleopDriveDefaultCommand();

        //you can run command groups
        gamepad1.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(IntakeSampleGroup.getPrepareIntakeSample());
        gamepad1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(IntakeSampleGroup.getIntakeSample());

        //or run commands from the subsystems
        gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(IntakeArm.getInstance().setPositionCommand(0.5));
    }

    @Override
    public void onInitLoop() {

    }

    @Override
    public void onPlay() {

    }

    @Override
    public void onPlayLoop() {

    }

    @Override
    public void onEnd() {

    }
}
