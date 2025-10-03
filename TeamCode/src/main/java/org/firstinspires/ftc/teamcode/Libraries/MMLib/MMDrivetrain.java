package org.firstinspires.ftc.teamcode.Libraries.MMLib;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.MMSubsystem;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.HoldPointCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import Ori.Coval.Logging.AutoLog;
import Ori.Coval.Logging.AutoLogPose2d;
import Ori.Coval.Logging.Logger.KoalaLog;

@AutoLog
public class MMDrivetrain extends MMSubsystem {

    public double slowModeRatioForward = 0.3;
    public double slowModeRatioLateral = 0.3;
    public double slowModeRatioRotation = 0.25;

    @IgnoreConfigurable
    private static MMDrivetrain instance;
    private static Follower follower;

    public static synchronized MMDrivetrain getInstance() {
        if (instance == null) {
            instance = new MMDrivetrainAutoLogged();
        }

        if (follower == null){
            follower = Constants.createFollower(MMRobot.getInstance().currentOpMode.hardwareMap);
        }
        return instance;
    }

    public Follower getFollower() {
        if (instance != null) {
            follower = Constants.createFollower(MMRobot.getInstance().currentOpMode.hardwareMap);
        }
        return follower;
    }

    public static void update() {
        if (instance != null) {
            follower.update();             //updates the follower
            Drawing.drawDebug(follower);
        }
    }

    public MMDrivetrain() {
        follower = Constants.createFollower(MMRobot.getInstance().currentOpMode.hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void resetHub() {
//        instance = null;
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
        return new HoldPointCommand(follower, temp, false);
    }

    public CommandBase turnToCommand(double radians) {
        return new HoldPointCommand(follower, new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(radians)), true);
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

    @AutoLogPose2d
    public double[] getAScopeTargetPose(){
        Pose TargetPose = follower.getClosestPose().getPose();
        return new double[]{TargetPose.getX(), TargetPose.getY(), TargetPose.getHeading()};
    }

    @AutoLogPose2d
    public double[] getAScopePose(){
        Pose pose = follower.getPose();
        return new double[]{pose.getX(), pose.getY(), pose.getHeading()};
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
