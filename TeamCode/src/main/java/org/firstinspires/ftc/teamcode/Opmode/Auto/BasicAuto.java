package org.firstinspires.ftc.teamcode.Opmode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMDrivetrain;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.AllianceColor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.FollowPathCommand;

@Autonomous
public class BasicAuto extends MMOpMode {

    private PathChain FROM_START_TO_SCORE, FROM_SCORE_TO_PARKING;

    private final Pose startPose = new Pose(104.000, 136.000,Math.toRadians(-90));
    private final Pose scorePose = new Pose(105.000, 105.000,Math.toRadians(50));
    private final Pose parkingPose = new Pose(90.000, 55.000,Math.toRadians(90));

    public void buildPaths() {
        FROM_START_TO_SCORE = MMDrivetrain.follower.pathBuilder()
                .addPath(new BezierLine(startPose,scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();
        FROM_SCORE_TO_PARKING = MMDrivetrain.follower.pathBuilder()
                .addPath(new BezierLine(scorePose,parkingPose))
                .setTangentHeadingInterpolation().build();
    }


    public BasicAuto() {
        super(OpModeType.NonCompetition.DEBUG, AllianceColor.BLUE);
    }

    @Override
    public void onInit() {
        MMDrivetrain.getInstance().update();
        MMDrivetrain.follower.setPose(startPose);
        buildPaths();
        Follower follower = MMDrivetrain.follower;

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, FROM_START_TO_SCORE),
                new FollowPathCommand(follower, FROM_SCORE_TO_PARKING)
        );
        autonomousSequence.schedule();
    }

    @Override
    public void onPlayLoop() {
        MMDrivetrain.getInstance().update();
        telemetry.update();
    }

}

