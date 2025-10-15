package org.firstinspires.ftc.teamcode.Libraries.pedroPathing;

import android.annotation.SuppressLint;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMUtils;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import java.util.Map;
import java.util.NavigableMap;
import java.util.Objects;
import java.util.TreeMap;

/**
 * This is the Pinpoint class. This class extends the Localizer superclass and is a
 * localizer that uses the two wheel odometry set up with the IMU to have more accurate heading
 * readings. The diagram below, which is modified from Road Runner, shows a typical set up.
 *
 */
public class PinpointVisionLocalizer implements Localizer {
    private final GoBildaPinpointDriver odo;
    private double previousHeading;
    private double totalHeading;
    private Pose startPose;
    private Pose currentVelocity;
    private Pose robotPose;
    private Pose pinpointPose;

    private final double kBufferDuration = 1.5;

    private final NavigableMap<Double, Pose> m_odometryPoseBuffer = new TreeMap<>();

    private final NavigableMap<Double, PinpointVisionLocalizer.VisionUpdate> m_visionUpdates = new TreeMap<>();

    private final double[] m_odometryStdDevs = new double[] {0.05, 0.05, 0.02};
    private final double[] m_visionMeasurementStdDevs = new double[] {0.5, 0.5, 0.2};

    private final double[] m_q = new double[3];
    private final double[][] m_visionK = new double[3][3];



    /**
     * This creates a new PinpointVisionLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public PinpointVisionLocalizer(HardwareMap map, PinpointVisionConstants constants){ this(map, constants, new Pose());}

    /**
     * This creates a new PinpointVisionLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map the HardwareMap
     * @param setStartPose the Pose to start from
     */
    @SuppressLint("NewApi")
    public PinpointVisionLocalizer(HardwareMap map, PinpointVisionConstants constants, Pose setStartPose){//TODO

        odo = map.get(GoBildaPinpointDriver.class,constants.hardwareMapName);
        setOffsets(constants.forwardPodY, constants.strafePodX, constants.distanceUnit);

        if(constants.yawScalar.isPresent()) {
            odo.setYawScalar(constants.yawScalar.getAsDouble());
        }

        if(constants.customEncoderResolution.isPresent()) {
            odo.setEncoderResolution(constants.customEncoderResolution.getAsDouble(), DistanceUnit.INCH);
        } else {
            odo.setEncoderResolution(constants.encoderResolution);
        }

        odo.setEncoderDirections(constants.forwardEncoderDirection, constants.strafeEncoderDirection);

        setStartPose(setStartPose);
        totalHeading = 0;
        robotPose = startPose;
        currentVelocity = new Pose();
        previousHeading = setStartPose.getHeading();

        setOdometryStdDevs(constants.odometryStdDevs[0], constants.odometryStdDevs[1], constants.odometryStdDevs[2]);
        setVisionMeasurementStdDevs(constants.visionMeasurementStdDevs[0], constants.visionMeasurementStdDevs[1], constants.visionMeasurementStdDevs[2]);
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        return robotPose;
    }

    /**
     * This returns the current pose estimate.
     *!!! THIS IS THE PINPOINT POSE WITHOUT THE VISION USE getPose for getting robot pose
     * @return returns the current pinpoint pose estimate as a Pose
     */
    public Pose getPinpointPosePose() {
        return pinpointPose;
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getAsVector();
    }

    /**
     * This sets the start pose. Since nobody should be using this after the robot has begun moving,
     * and due to issues with the PinpointVisionLocalizer, this is functionally the same as setPose(Pose).
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        if (!Objects.equals(startPose, new Pose()) && startPose != null) {
            Pose currentPose = robotPose.rotate(-startPose.getHeading(), false).minus(startPose);
            setPose(setStart.plus(currentPose.rotate(setStart.getHeading(), false)));
        } else {
            setPose(setStart);
        }

        this.startPose = setStart;
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        odo.setPosition(PoseConverter.poseToPose2D(setPose, PedroCoordinates.INSTANCE));
        pinpointPose = setPose;
        robotPose = setPose;
        previousHeading = setPose.getHeading();
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
    }

    /**
     * This updates the total heading of the robot. The Pinpoint handles all other updates itself.
     */
    @Override
    public void update() {
        odo.update();
        Pose currentPinpointPose = PoseConverter.pose2DToPose(odo.getPosition(), PedroCoordinates.INSTANCE);
        totalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), previousHeading);
        previousHeading = currentPinpointPose.getHeading();
        currentVelocity = new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeading(AngleUnit.RADIANS));
        pinpointPose = currentPinpointPose;

        // sample and store odometry-only pose at this timestamp (seconds)
        double nowSeconds = System.nanoTime() / 1e9;
        Pose odomPose = pinpointPose.copy();
        m_odometryPoseBuffer.put(nowSeconds, odomPose);

        // clean up old odometry samples
        cleanUpOdometryBuffer(nowSeconds);

        // update current pose estimate by applying the latest vision compensation (if any)
        if (m_visionUpdates.isEmpty()) {
            robotPose = odomPose.copy();
        } else {
            PinpointVisionLocalizer.VisionUpdate latestVU = m_visionUpdates.get(m_visionUpdates.lastKey());
            assert latestVU != null;
            robotPose = latestVU.compensate(odomPose);
        }
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the Y encoder value as none of the odometry tuners are required for this localizer
     * @return returns the Y encoder value
     */
    @Override
    public double getForwardMultiplier() {
        return odo.getEncoderY();
    }

    /**
     * This returns the X encoder value as none of the odometry tuners are required for this localizer
     * @return returns the X encoder value
     */
    @Override
    public double getLateralMultiplier() {
        return odo.getEncoderX();
    }

    /**
     * This returns either the factory tuned yaw scalar or the yaw scalar tuned by yourself.
     * @return returns the yaw scalar
     */
    @Override
    public double getTurningMultiplier() {
        return odo.getYawScalar();
    }

    /**
     * This sets the offsets and converts inches to millimeters
     * @param xOffset How far to the side from the center of the robot is the x-pod? Use positive values if it's to the left and negative if it's to the right.
     * @param yOffset How far forward from the center of the robot is the y-pod? Use positive values if it's forward and negative if it's to the back.
     * @param unit The units that the measurements are given in
     */
    private void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        odo.setOffsets(xOffset, yOffset, unit);
    }

    /**
     * This resets the IMU. Does not change heading estimation.
     */
    @Override
    public void resetIMU() {
        resetPinpoint();
    }

    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    /**
     * This resets the pinpoint.
     */
    private void resetPinpoint() {
        odo.resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * This recalibrates the Pinpoint. It will take 0.25 seconds to recalibrate, and the robot must be still
     */
    public void recalibrate() {
        odo.recalibrateIMU();
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns whether the robot's position is NaN
     */
    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }

    /**
     * This returns the GoBildaPinpointDriver object used by this localizer, in case you want to
     * access any of its methods directly.
     *
     * @return returns the GoBildaPinpointDriver object used by this localizer
     */
    public GoBildaPinpointDriver getPinpoint() {
        return odo;
    }

    // -----------VISION----------------

    /**
     * Set the standard deviations (x, y, heading) for the odometry uncertainty.
     */
    public void setOdometryStdDevs(double xStd, double yStd, double headingStd) {
        m_odometryStdDevs[0] = xStd;
        m_odometryStdDevs[1] = yStd;
        m_odometryStdDevs[2] = headingStd;
        for (int i = 0; i < 3; ++i) {
            m_q[i] = m_odometryStdDevs[i] * m_odometryStdDevs[i];
        }
        recomputeVisionK();
    }

    /**
     * Set the standard deviations (x, y, heading) for vision measurement uncertainty.
     */
    public void setVisionMeasurementStdDevs(double xStd, double yStd, double headingStd) {
        m_visionMeasurementStdDevs[0] = xStd;
        m_visionMeasurementStdDevs[1] = yStd;
        m_visionMeasurementStdDevs[2] = headingStd;
        recomputeVisionK();
    }

    private void recomputeVisionK() {
        // r = vision variance
        double[] r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = m_visionMeasurementStdDevs[i] * m_visionMeasurementStdDevs[i];
        }

        // zero the matrix first (ensures off-diagonals are cleared)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m_visionK[i][j] = 0.0;
            }
        }

        // closed-form diagonal Kalman-like gain used by WPILib:
        // k = q / ( q + sqrt(q * r) ), placed on the diagonal
        for (int i = 0; i < 3; ++i) {
            double k;
            if (m_q[i] == 0.0) {
                k = 0.0;
            } else {
                k = m_q[i] / (m_q[i] + Math.sqrt(m_q[i] * r[i]));
            }
            m_visionK[i][i] = k;
        }
    }

    private void cleanUpOdometryBuffer(double nowSeconds) {
        double cutoff = nowSeconds - kBufferDuration;
        m_odometryPoseBuffer.headMap(cutoff, false).clear();
        cleanUpVisionUpdates();
    }

    /**
     * Return the odometry-only pose at given timestamp (seconds) by linear interpolation.
     * Returns null if buffer empty.
     */
    public Pose getOdometrySample(double timestampSeconds) {
        if (m_odometryPoseBuffer.isEmpty()) {
            return null;
        }

        double oldest = m_odometryPoseBuffer.firstKey();
        double newest = m_odometryPoseBuffer.lastKey();

        // clamp
        timestampSeconds = Math.max(oldest, Math.min(newest, timestampSeconds));

        if (m_odometryPoseBuffer.containsKey(timestampSeconds)) {
            return m_odometryPoseBuffer.get(timestampSeconds).copy();
        }

        Map.Entry<Double, Pose> floor = m_odometryPoseBuffer.floorEntry(timestampSeconds);
        Map.Entry<Double, Pose> ceil = m_odometryPoseBuffer.ceilingEntry(timestampSeconds);

        if (floor == null) return ceil.getValue().copy();
        if (ceil == null) return floor.getValue().copy();

        if (floor.getKey().equals(ceil.getKey())) {
            return floor.getValue().copy();
        }

        double t0 = floor.getKey();
        double t1 = ceil.getKey();
        double frac = (timestampSeconds - t0) / (t1 - t0);

        Pose p0 = floor.getValue();
        Pose p1 = ceil.getValue();

        // linearly interpolate x,y; interpolate heading using smallest angle difference
        double ix = p0.getX() + frac * (p1.getX() - p0.getX());
        double iy = p0.getY() + frac * (p1.getY() - p0.getY());
        double dtheta = MathFunctions.getSmallestAngleDifference(p0.getHeading(), p1.getHeading());
        double iheading = p0.getHeading() + frac * dtheta;

        return new Pose(ix, iy, iheading);
    }

    /**
     * Samples the vision-compensated pose at timestampSeconds. Returns null if buffer empty.
     */
    public Pose sampleAt(double timestampSeconds) {
        if (m_odometryPoseBuffer.isEmpty()) {
            return null;
        }

        // clamp timestamp within odometry buffer range
        double oldest = m_odometryPoseBuffer.firstKey();
        double newest = m_odometryPoseBuffer.lastKey();
        timestampSeconds = Math.max(oldest, Math.min(newest, timestampSeconds));

        // If there are no applicable vision updates before this timestamp, just return odometry sample
        if (m_visionUpdates.isEmpty() || timestampSeconds < m_visionUpdates.firstKey()) {
            return getOdometrySample(timestampSeconds);
        }

        // get the latest vision update <= timestamp
        double floorTimestamp = m_visionUpdates.floorKey(timestampSeconds);
        PinpointVisionLocalizer.VisionUpdate visionUpdate = m_visionUpdates.get(floorTimestamp);

        Pose odomSample = getOdometrySample(timestampSeconds);

        if (odomSample == null) return null;
        return visionUpdate.compensate(odomSample);
    }

    /** Removes vision updates that are too old to matter relative to the odometry buffer. */
    private void cleanUpVisionUpdates() {
        if (m_odometryPoseBuffer.isEmpty()) return;

        double oldestOdometryTimestamp = m_odometryPoseBuffer.firstKey();
        if (m_visionUpdates.isEmpty() || oldestOdometryTimestamp < m_visionUpdates.firstKey()) {
            return;
        }

        // newest needed vision update at or before oldest odometry timestamp
        double newestNeeded = m_visionUpdates.floorKey(oldestOdometryTimestamp);
        // remove strictly before newestNeeded
        m_visionUpdates.headMap(newestNeeded, false).clear();
    }

    /**
     * Add a vision measurement (pose) with a timestampSeconds (seconds epoch). This will:
     *  - reject if measurement too old for buffer,
     *  - compute the difference between vision pose and current vision-compensated pose at that time,
     *  - scale it by a per-axis Kalman-like gain,
     *  - store a VisionUpdate and update the current pose estimate.
     * Usage: when your vision system reports a pose, call:
     *   PinpointVisionLocalizer.addVisionMeasurement(visionPose, System.nanoTime()/1e9);
     */
    public void addVisionMeasurement(Pose visionRobotPoseMeters, double timestampSeconds) {
        // Step 0: reject if too old (outside buffer window)
        if (m_odometryPoseBuffer.isEmpty()
            || m_odometryPoseBuffer.lastKey() - kBufferDuration > timestampSeconds) {
            return;
        }

        // Step 1: clean old entries
        cleanUpVisionUpdates();

        // Step 2: odometry pose at the moment the vision measurement was made
        Pose odomSample = getOdometrySample(timestampSeconds);
        if (odomSample == null) return;

        // Step 3: vision-compensated pose at measurement time
        Pose visionSample = sampleAt(timestampSeconds);
        if (visionSample == null) return;

        // Step 4: compute delta (twist) from visionSample to measured vision pose
        double dx = visionRobotPoseMeters.getX() - visionSample.getX();
        double dy = visionRobotPoseMeters.getY() - visionSample.getY();
        double raw = visionRobotPoseMeters.getHeading() - visionSample.getHeading();
        double dtheta = Math.copySign(
            MathFunctions.getSmallestAngleDifference(visionSample.getHeading(),
                visionRobotPoseMeters.getHeading()),
            raw);

        // Rotate delta into the local (visionSample) frame:
        double theta = visionSample.getHeading();
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        // R(-theta) * [dx_f; dy_f] gives local-frame translation
        double dx_local =  cos * dx + sin * dy;
        double dy_local = -sin * dx + cos * dy;

        // Step 5: scale by full 3x3 gain matrix (m_visionK is double[3][3], row-major)
        double[] localDelta = new double[]{dx_local, dy_local, dtheta};
        double[] k_times_twist = MMUtils.mat3x3MulVec(m_visionK, localDelta);

        double scaled_dx_local = k_times_twist[0];
        double scaled_dy_local = k_times_twist[1];
        double scaled_dtheta   = k_times_twist[2];

        // Step 6: rotate scaled local corrections back to field frame:
        double corr_x_f = cos * scaled_dx_local - sin * scaled_dy_local;
        double corr_y_f = sin * scaled_dx_local + cos * scaled_dy_local;

        double currentUnwrappedHeading = visionSample.getHeading() + scaled_dtheta;
        double wrappedAngle =Math.atan2(
            Math.sin(currentUnwrappedHeading), Math.cos(currentUnwrappedHeading));

        // Apply correction to visionSample in field coords
        Pose correctedVisionPose = new Pose(
            visionSample.getX() + corr_x_f,
            visionSample.getY() + corr_y_f,
            wrappedAngle
        );

        // Step 7: record the vision update and remove later vision measurements
        PinpointVisionLocalizer.VisionUpdate visionUpdate = new PinpointVisionLocalizer.VisionUpdate(correctedVisionPose, odomSample);
        m_visionUpdates.put(timestampSeconds, visionUpdate);
        m_visionUpdates.tailMap(timestampSeconds, false).clear();

        // Step 8: update latest pose estimate by compensating current odometry pose
        robotPose = visionUpdate.compensate(pinpointPose);
    }


    /**
     * Represents a vision update record. The record contains the vision-compensated pose estimate as
     * well as the corresponding odometry pose estimate.
     */
    private static final class VisionUpdate {
        // The vision-compensated pose estimate.
        private final Pose visionPose;

        // The pose estimated based solely on odometry.
        private final Pose odometryPose;

        /**
         * Constructs a vision update record with the specified parameters.
         *
         * @param visionPose The vision-compensated pose estimate.
         * @param odometryPose The pose estimate based solely on odometry.
         */
        private VisionUpdate(Pose visionPose, Pose odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }

        /**
         * Returns the vision-compensated version of the pose. Specifically, changes the pose from being
         * relative to this record's odometry pose to being relative to this record's vision pose.
         *
         * @param pose The pose to compensate.
         * @return The compensated pose.
         */
        public Pose compensate(Pose pose) {
            Pose delta = pose.minus(this.odometryPose);
            return this.visionPose.plus(delta);
        }
    }
}
