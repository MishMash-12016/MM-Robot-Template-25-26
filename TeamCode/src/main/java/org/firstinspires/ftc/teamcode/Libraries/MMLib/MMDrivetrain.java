package org.firstinspires.ftc.teamcode.Libraries.MMLib;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.HoldPointCommand;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import Ori.Coval.Logging.AutoLog;

@AutoLog
public class MMDrivetrain extends SubsystemBase {

    public double slowModeRatioForward = 0.3;
    public double slowModeRatioLateral = 0.3;
    public double slowModeRatioRotation = 0.25;

    @IgnoreConfigurable
    private static MMDrivetrain instance;
    public static Follower follower;

    public static synchronized MMDrivetrain getInstance() {
        if (instance == null) {
            instance = new MMDrivetrain();
        }
        return instance;
    }

    public Follower getFollower() {
        if (instance != null) {
            follower = Constants.createFollower(MMRobot.getInstance().currentOpMode.hardwareMap);
        }
        return follower;
    }

    public void update() {
        if (instance != null) {
            follower.update();             //updates the follower

        }
    }

    public MMDrivetrain() {
        follower = Constants.createFollower(MMRobot.getInstance().currentOpMode.hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    public CommandBase holdPointCommand(Pose pose) {
        CommandBase holdPointCommand = new HoldPointCommand(follower, pose, false);
        holdPointCommand.addRequirements(this);

        return holdPointCommand;
    }

    public CommandBase followPathCommand(Path path, boolean holdEnd) {
        CommandBase followPathCommand = new FollowPathCommand(follower, path, holdEnd);
        followPathCommand.addRequirements(this);

        return followPathCommand;
    }

    public CommandBase followPathCommand(Path path) {
        return this.followPathCommand(path, true);
    }

    public CommandBase followPathCommand(PathChain pathChain) {
        CommandBase followPathCommand = new FollowPathCommand(follower, pathChain, true);
        followPathCommand.addRequirements(this);

        return followPathCommand;
    }

    public CommandBase driveCommand(DoubleSupplier forwardDrive, DoubleSupplier lateralDrive, DoubleSupplier heading, BooleanSupplier slowMode) {
        return this.driveCommand(forwardDrive, lateralDrive, heading, false, slowMode);
    }

    public CommandBase driveCommand(DoubleSupplier forwardDrive, DoubleSupplier lateralDrive, DoubleSupplier heading, boolean robotCentric, BooleanSupplier slowMode) {
        return (CommandBase) new RunCommand(() -> {

            if (slowMode.getAsBoolean()) {
                follower.setTeleOpDrive(//TODO: add variables for the math.pow
                        Math.pow(forwardDrive.getAsDouble(), 5) * slowModeRatioForward,
                        Math.pow(lateralDrive.getAsDouble(), 5) * slowModeRatioLateral,
                        Math.pow(heading.getAsDouble(), 1) * slowModeRatioRotation,
                        robotCentric);
            } else {
                //TODO: add math.pow with variables
                follower.setTeleOpDrive(forwardDrive.getAsDouble(), lateralDrive.getAsDouble(), heading.getAsDouble(), robotCentric);
            }

            follower.update();
        }, this)
                .beforeStarting(() -> {
                    follower.startTeleopDrive();
                    follower.setMaxPower(2);//TODO:testing to see if this works and if it is the right way to do this
                }).whenFinished(() -> follower.setMaxPower(1));
    }

    public CommandBase turnCommand(double radians, boolean isLeft) {
        Pose temp = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading() + (isLeft ? radians : -radians));
        return this.holdPointCommand(temp);
    }

    public CommandBase turnToCommand(double radians) {
        return this.holdPointCommand(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(radians)));
    }

    public CommandBase turnToDegreesCommand(double degrees) {
        return this.turnToCommand(Math.toRadians(degrees));
    }

    public CommandBase turnDegreesCommand(double degrees, boolean isLeft) {
        return this.turnCommand(Math.toRadians(degrees), isLeft);
    }

    public void resetYaw() {
        Pose pose = follower.getPose();
        pose.setHeading(0);
        follower.setPose(pose);
    }

    /**
     * enables the Default Command(the default command is the drive field centric command)
     */
    public void enableTeleopDriveDefaultCommand(BooleanSupplier slowMode) {
        MMRobot mmRobot = MMRobot.getInstance();
        setDefaultCommand(driveCommand(
                () -> mmRobot.gamepadEx1.getLeftY(),
                () -> -mmRobot.gamepadEx1.getLeftX(),
                () -> -mmRobot.gamepadEx1.getRightX(),
                false, slowMode)
        );
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void setPose(double x, double y, double heading) {
        this.setPose(new Pose(x, y, heading));
    }

    /**
     * disables the Default Command(the default command is the drive field centric command)
     */
    public void disableTeleopDriveDefaultCommand() {
        setDefaultCommand(new RunCommand(() -> {
        }, this));
    }

    public void setSlowModeRatioForward(double slowModeRatioForward) {
        this.slowModeRatioForward = slowModeRatioForward;
    }

    public void setSlowModeRatioLateral(double slowModeRatioLateral) {
        this.slowModeRatioLateral = slowModeRatioLateral;
    }

    public void setSlowModeRatioRotation(double slowModeRatioRotation) {
        this.slowModeRatioRotation = slowModeRatioRotation;
    }
}
