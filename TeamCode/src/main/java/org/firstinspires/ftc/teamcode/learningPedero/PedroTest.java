package org.firstinspires.ftc.teamcode.learningPedero;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.RobotConstants;

@Autonomous
public class PedroTest extends MMOpMode {

    private Follower follower;
    private PathBuilder builder;

    public PedroTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {
        follower = MMSystems.getInstance().initializeFollower(RobotConstants.basketStartPose);
        builder = follower.pathBuilder();
    }

    @Override
    public void onInitLoop() {}

    @Override
    public void onPlay() {
        schedule(
                new FollowPathCommand(follower,
                        builder.addPath(
                                new BezierCurve(
                                        new Point(RobotConstants.basketStartPose),
                                        new Point(12, 105),
                                        new Point(RobotConstants.basketPose)
                                )
                        ).setLinearHeadingInterpolation(
                                RobotConstants.basketStartPose.getHeading(),
                                RobotConstants.basketPose.getHeading()
                        ).build()
                ),
                new InstantCommand(),
                new FollowPathCommand(follower,
                        builder.addPath(
                                new BezierCurve(
                                        new Point(RobotConstants.basketPose),
                                        new Point(25, 131),
                                        new Point(RobotConstants.yellowSampleClose)
                                )
                        ).setLinearHeadingInterpolation(
                                RobotConstants.basketPose.getHeading(),
                                RobotConstants.yellowSampleClose.getHeading()
                        ).build()
                )
        );
    }

    @Override
    public void onPlayLoop() {
        follower.update();
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
    }

    @Override
    public void onEnd() {}

}
