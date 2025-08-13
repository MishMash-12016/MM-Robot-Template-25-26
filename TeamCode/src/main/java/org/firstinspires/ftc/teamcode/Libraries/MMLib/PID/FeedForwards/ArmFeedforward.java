// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.FeedForwards;

/**
 * A helper class that computes feedforward outputs for a simple arm (modeled as a motor acting
 * against the force of gravity on a beam suspended at an angle).
 */
public class ArmFeedforward extends SimpleMotorFeedforward{

    /** The gravity gain, in volts. */
    public double kg;

    /**
     * Creates a new ArmFeedforward with the specified gains. Units of the gain values will dictate
     * units of the computed feedforward.
     *
     * @param ks The static gain.
     * @param kg The gravity gain.
     * @param kv The velocity gain.
     * @param ka The acceleration gain.
     * @throws IllegalArgumentException for kv &lt; zero.
     * @throws IllegalArgumentException for ka &lt; zero.
     */
    public ArmFeedforward(double ks, double kg, double kv, double ka) {
        super(ks, kv, ka);
        this.kg = kg;
    }

    /**
     * Creates a new ArmFeedforward with the specified gains. Acceleration gain is defaulted to zero.
     * Units of the gain values will dictate units of the computed feedforward.
     *
     * @param ks The static gain.
     * @param kg The gravity gain.
     * @param kv The velocity gain.
     */
    public ArmFeedforward(double ks, double kg, double kv) {
        this(ks, kg, kv, 0);
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param positionRadians The position (angle) setpoint. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
     *     your encoder does not follow this convention, an offset should be added.
     * @param velocityRadPerSec The velocity setpoint.
     * @param accelRadPerSecSquared The acceleration setpoint.
     * @return The computed feedforward.
     */
    public double calculate(
            double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
        return ks * Math.signum(velocityRadPerSec)
                + kg * Math.cos(positionRadians)
                + kv * velocityRadPerSec
                + ka * accelRadPerSecSquared;
    }

    /**
     * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to be
     * zero).
     *
     * @param positionRadians The position (angle) setpoint. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
     *     your encoder does not follow this convention, an offset should be added.
     * @param velocity The velocity setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double positionRadians, double velocity) {
        return calculate(positionRadians, velocity, 0);
    }

    @Override
    public double getKg() {
        return kg;
    }

    @Override
    public void setKg(double kg) {
        this.kg = kg;
    }
}
