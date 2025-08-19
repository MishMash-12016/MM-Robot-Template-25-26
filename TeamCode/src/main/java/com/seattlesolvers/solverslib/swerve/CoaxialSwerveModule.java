package com.seattlesolvers.solverslib.swerve;

import static com.seattlesolvers.solverslib.util.MathUtils.normalizeRadians;

import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Deprecated
public class CoaxialSwerveModule {
    private final MotorEx motor;
    private final CRServoEx swervo;
    private final Vector2d offset;
    private final double maxSpeed;
    private final PIDFController swervoPIDF;
    private final double tangentialAngle;
    private final double circumference;
    private Vector2d targetVelocity = new Vector2d();

    /**
     * The constructor that sets up the swerve module object.
     *
     * @param motor the SolversMotor for the swerve module.
     * @param swervo the SolversAxonServo for the pod rotation, with the proper absolute encoder and angle offset so that the wheel is moving the robot forward when motor power is positive.
     * @param offset the offset of the center of the wheel/pod from the center of the robot, in inches.
     * @param maxSpeed the maximum linear speed of the wheel/pod in inches/second.
     */
    public CoaxialSwerveModule(MotorEx motor, CRServoEx swervo, Vector2d offset, double maxSpeed, double[] swervoPIDFCoefficients) {
        this.motor = motor;
        this.swervo = swervo;
        this.offset = offset;
        this.maxSpeed = maxSpeed;
        this.swervoPIDF = new PIDFController(swervoPIDFCoefficients[0], swervoPIDFCoefficients[1], swervoPIDFCoefficients[2], swervoPIDFCoefficients[3]);

        tangentialAngle = offset.angle() + (Math.signum(offset.getX()) * Math.PI/2);
        circumference = offset.magnitude() * Math.PI * 2;
    }

    /**
     * The main kinematics for robot centric module movements. Made so that it is easy to follow/understand: <a href="https://www.desmos.com/calculator/8sm94so6ud">see Desmos link</a>.
     *
     * @param translational the translational vector the robot is to follow in inches/second.
     * @param rotational the rotational vector the robot is to follow in radians/second.
     * @return returns normalized vector for module to follow from target translational and rotational velocity parameters.
     */
    public Vector2d calculateVectorRobotCentric(Vector2d translational, double rotational) {
        // Turning vector
        // See calculations for tangentialAngle and circumference in constructor, as they don't need to be recalculated every time
        double turningVectorMagnitude = circumference * rotational / (Math.PI * 2);
        Vector2d turningVector = new Vector2d(Math.cos(tangentialAngle) * turningVectorMagnitude, Math.sin(tangentialAngle) * turningVectorMagnitude);

        // Final vector
        // Averaging turning and translational vectors
        Vector2d finalVector = turningVector.plus(translational).div(2);

        // Normalizing vector
        double scalar = Math.max(1, finalVector.magnitude() / maxSpeed);
        finalVector.div(scalar);

        return finalVector;
    }

    /**
     * Sets the target velocity
     * @param velocity the vector that represents the velocity the pod/wheel has to follow in inches/second.
     */
    private void setTargetVelocity(Vector2d velocity) {
        targetVelocity = velocity;
    }

    /**
     * Updates the module/hardware to follow the known, previous target velocity set in the object.
     */
    private void updateModule() {
        // Wheel flipping optimization (if its quicker to swap motor direction and rotate the pod less, then do that)
        boolean wheelFlipped = false;
        double error = normalizeRadians(targetVelocity.angle() - swervo.getAbsoluteEncoder().getCurrentPosition(), true);
        if (Math.abs(error) > Math.PI/2) {
            error += Math.PI * -Math.signum(error);
            wheelFlipped = true;
        }

        // Set wheel speed
        if (wheelFlipped) {
            motor.set(-targetVelocity.magnitude() / maxSpeed);
        } else {
            motor.set(targetVelocity.magnitude() / maxSpeed);
        }

        // Set swervo speed for pod rotation
        swervo.set(swervoPIDF.calculate(0, error));
    }

    /**
     * Updates the module/hardware to follow the target velocity passed in the parameter.
     * @param velocity the velocity the pod/wheel should have, robot centric and in inches/second.
     */
    private void updateModuleWithVelocity(Vector2d velocity) {
        setTargetVelocity(targetVelocity);
        updateModule();
    }

    /**
     * Updates the module/hardware to follow the overall robot centric velocities passed in the parameters with the kinematics.
     * @param translational the translational velocity the robot should have, robot centric and in inches/second.
     * @param rotational the rotational velocity the robot should have, in radians/second.
     */
    private void updateModuleWithRobotCentricKinematics(Vector2d translational, double rotational) {
        updateModuleWithVelocity(calculateVectorRobotCentric(translational, rotational));
    }
}
