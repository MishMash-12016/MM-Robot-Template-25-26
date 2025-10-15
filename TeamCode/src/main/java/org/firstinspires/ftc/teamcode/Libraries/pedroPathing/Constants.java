package org.firstinspires.ftc.teamcode.Libraries.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .forwardZeroPowerAcceleration(-36.84)
            .lateralZeroPowerAcceleration(-78.68)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.005, 0.027))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.09, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.0006, 0.6, 0.05));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("fl")
            .leftRearMotorName("bl")
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .useBrakeModeInTeleOp(true)
            .xVelocity(79.71)
            .yVelocity(36.125);

    public static PinpointVisionConstants localizerConstants = new PinpointVisionConstants()
            .forwardPodY(-10 / 2.54)
            .strafePodX(-14 / 2.54)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .odometryStdDevs(0.05, 0.05, 0.02)
            .visionMeasurementStdDevs(0.5, 0.5, 0.2);


    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            1.2,
            0.1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(new PinpointVisionLocalizer(hardwareMap, localizerConstants))
                .pathConstraints(pathConstraints)
                .build();
    }
}
