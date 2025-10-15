package org.firstinspires.ftc.teamcode.Libraries.pedroPathing;

import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.os.Build;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;

/**
 * This is the PinpointVisionConstants class. It holds many constants and parameters for the Pinpoint Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

@TargetApi(Build.VERSION_CODES.N)
public class PinpointVisionConstants {
    //TODO: OdometryStdDevs
    //TODO: VisionMeasurementStdDevs

    /** The Y Offset of the Forward Encoder (Deadwheel) from the center of the robot in DistanceUnit
     * @see #distanceUnit
     * Default Value: 1 */
    public  double forwardPodY = 1;

    /** The X Offset of the Strafe Encoder (Deadwheel) from the center of the robot in DistanceUnit
     * @see #distanceUnit
     * Default Value: -2.5 */
    public  double strafePodX = -2.5;

    /** The Unit of Distance that the Pinpoint uses to measure distance
     * Default Value: DistanceUnit.INCH */
    public  DistanceUnit distanceUnit = DistanceUnit.INCH;

    /** The name of the Pinpoint in the hardware map (name of the I2C port it is plugged into)
     * Default Value: "pinpoint" */
    public  String hardwareMapName = "pinpoint";

    /** Custom Yaw Scalar for the Pinpoint (overrides the calibration of the Pinpoint) */
    @SuppressLint("NewApi")
    public OptionalDouble yawScalar = OptionalDouble.empty();

    /** The Encoder Resolution for the Pinpoint. Used by default, but can be changed to a custom resolution.
     * Default Value: GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD */
    public  GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /** The Encoder Resolution for the Pinpoint. Unused by default, but can be used if you want to use a custom encoder resolution. */
    @SuppressLint("NewApi")
    public OptionalDouble customEncoderResolution = OptionalDouble.empty();

    /** The Encoder Direction for the Forward Encoder (Deadwheel)
     * Default Value: GoBildaPinpointDriver.EncoderDirection.REVERSED */
    public  GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    /** The Encoder Direction for the Strafe Encoder (Deadwheel)
     * Default Value: GoBildaPinpointDriver.EncoderDirection.FORWARD */
    public  GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    /**odometryStdDevs Standard deviations of the odometry measurements. Increase these
     * numbers to trust global measurements from odometry less. This array is in the form [x, y,
     * theta]ᵀ, with units in inches and radians.*/
    public double[] odometryStdDevs = new double[] {0.05, 0.05, 0.02};

    /**visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
     * numbers to trust global measurements from vision less. This array is in the form [x, y,
     * theta]ᵀ, with units in inches and radians.*/
    public double[] visionMeasurementStdDevs = new double[] {0.5, 0.5, 0.2};

    /**
     * This creates a new PinpointVisionConstants with default values.
     */
    public PinpointVisionConstants() {
        defaults();
    }

    public PinpointVisionConstants forwardPodY(double forwardPodY) {
        this.forwardPodY = forwardPodY;
        return this;
    }

    public PinpointVisionConstants strafePodX(double strafePodX) {
        this.strafePodX = strafePodX;
        return this;
    }

    public PinpointVisionConstants distanceUnit(DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
        return this;
    }

    public PinpointVisionConstants hardwareMapName(String hardwareMapName) {
        this.hardwareMapName = hardwareMapName;
        return this;
    }

    public PinpointVisionConstants yawScalar(double yawScalar) {
        this.yawScalar = OptionalDouble.of(yawScalar);
        return this;
    }

    public PinpointVisionConstants encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution) {
        this.encoderResolution = encoderResolution;
        return this;
    }

    public PinpointVisionConstants customEncoderResolution(double customEncoderResolution) {
        this.customEncoderResolution = OptionalDouble.of(customEncoderResolution);
        return this;
    }

    public PinpointVisionConstants forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection) {
        this.forwardEncoderDirection = forwardEncoderDirection;
        return this;
    }

    public PinpointVisionConstants strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection) {
        this.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }

    public PinpointVisionConstants odometryStdDevs(double xStd, double yStd, double headingStd) {
        odometryStdDevs[0] = xStd;
        odometryStdDevs[1] = yStd;
        odometryStdDevs[2] = headingStd;
        return this;
    }

    public PinpointVisionConstants visionMeasurementStdDevs(double xStd, double yStd, double headingStd) {
        visionMeasurementStdDevs[0] = xStd;
        visionMeasurementStdDevs[1] = yStd;
        visionMeasurementStdDevs[2] = headingStd;
        return this;
    }

    public void defaults() {
        forwardPodY = 1;
        strafePodX = -2.5;
        distanceUnit = DistanceUnit.INCH;
        hardwareMapName = "pinpoint";
        yawScalar = OptionalDouble.empty();
        encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        customEncoderResolution = OptionalDouble.empty();
        forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        odometryStdDevs = new double[] {0.05, 0.05, 0.02};
        visionMeasurementStdDevs = new double[] {0.5, 0.5, 0.2};
    }
}
