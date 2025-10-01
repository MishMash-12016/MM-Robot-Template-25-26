package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems;


import android.util.Size;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import Ori.Coval.Logging.AutoLog;
import Ori.Coval.Logging.AutoLogOutput;

@AutoLog
public class WebcamSubsystem extends SubsystemBase {

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    private double distance = 0;
    private double angle = 0;
    public WebcamSubsystem(){
        initAprilTag();
    }

    public void update(){
            for (AprilTagDetection detection : aprilTag.getDetections()) {
                distance = detection.ftcPose.range;
                angle = detection.ftcPose.bearing;
            }
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .build();

        //aprilTag.setDecimation(3);
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(MMRobot.getInstance().currentOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, true);
        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();
    }

    @Override
    public void periodic() {
        super.periodic();
        update();
    }

    @AutoLogOutput
    public double getDistance() {
        return distance;
    }

    @AutoLogOutput
    public double getAngle() {
        return angle;
    }
}
