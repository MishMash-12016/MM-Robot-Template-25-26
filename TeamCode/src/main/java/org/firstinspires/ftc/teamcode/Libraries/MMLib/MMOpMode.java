package org.firstinspires.ftc.teamcode.Libraries.MMLib;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;


import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.AllianceColor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.AllianceSide;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import Ori.Coval.Logging.AutoLogManager;
import Ori.Coval.Logging.Logger.KoalaLog;

import org.firstinspires.ftc.teamcode.MMRobot;


/**
 * this class represents a wrapper for the default Teleop.
 * <p>
 * if it is used while calling the side and/or color val it initializes the MMRobot with the {@link OpModeType.Competition#TELEOP Teleop} type.
 * u might also use the constructor that only requires a {@link OpModeType.NonCompetition  NonComp} type,
 * this constructor let's u insert the type of {@link OpModeType.NonCompetition  NonComp} opmode u would like to use.
 * there are explanations in {@link OpModeType} that explains the 3 options u have.
 * there is the {@link OpModeType.NonCompetition#DEBUG Debug},
 * {@link OpModeType.NonCompetition#EXPERIMENTING_NO_EXPANSION Experimenting Without Expansion}.
 */
public abstract class MMOpMode extends LinearOpMode {

    private final MMRobot mmRobot = MMRobot.getInstance();

    public OpModeType opModeType = null;

    public AllianceColor allianceColor;
    public AllianceSide allianceSide;

    private final List<Runnable> runOnInit = new ArrayList<>();
    private final List<Command> commandsOnRun = new ArrayList<>();

    /**
     * use this to choose a {@link OpModeType.NonCompetition NonComp} opmode.
     *
     * @param opModeType which opmode to activate
     */
    public MMOpMode(OpModeType opModeType) {
        this.opModeType = opModeType;
    }

    private void robotInit() {
        mmRobot.currentOpMode = this;
        MMRobot.getInstance().initializeSystems(opModeType);
        KoalaLog.setup(hardwareMap);//TODO: maybe move this
    }

    public abstract void onInit();

    public abstract void onInitLoop();

    public abstract void onPlay();

    /**
     * Updates the {@link CommandScheduler}, {@link org.firstinspires.ftc.robotcore.external.Telemetry Telemetry}
     * and {@link org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub Control & Expansion Hub} on Play Loop.
     */
    public void onPlayLoopUpdates() {
        CommandScheduler.getInstance().run();                     //runs the scheduler

        MMRobot.getInstance().controlHub.pullBulkData();        //updates the controlHub sensors
        if (MMRobot.getInstance().expansionHub != null) {
            MMRobot.getInstance().expansionHub.pullBulkData();  //updates the expansionHub sensors
        }

        MMDrivetrain.update();

        telemetry.update();                                       //updates the telemetry

        FtcDashboard.getInstance().getTelemetry().update();       //updates the dashboard

        AutoLogManager.periodic();
    }

    public abstract void onPlayLoop();

    public abstract void onEnd();

    /**
     * Cancels all previous commands and deletes the {@link MMRobot Robot Singleton}
     */
    public void reset() {
        CommandScheduler.getInstance().cancelAll();
    }

    public void addRunnableOnInit(Runnable... runOnInit) {
        this.runOnInit.addAll(Arrays.asList(runOnInit));
    }

    public void addCommandsOnRun(Command... commandsOnRun) {
        this.commandsOnRun.add(new InstantCommand().andThen(commandsOnRun));
        /*this was in order to solve the commandScheduler problem
          the problem was that the scheduler for some reason always ran the first instant command even tho it wasn't on yet*/
    }

    private void scheduleCommandsAndRun() {
        for (Runnable runnable : runOnInit) {
            runnable.run();
        }

        for (Command command : commandsOnRun) {
            command.schedule();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robotInit();
        onInit();
        scheduleCommandsAndRun();

        try {
            while (opModeInInit()) {
                onInitLoop();
            }
            onPlay();
            while (!isStopRequested() && opModeIsActive()) {
                onPlayLoopUpdates();
                onPlayLoop();
            }
        } finally {
            try {
                onEnd();
            } finally {
                reset();
            }
        }
    }


}
