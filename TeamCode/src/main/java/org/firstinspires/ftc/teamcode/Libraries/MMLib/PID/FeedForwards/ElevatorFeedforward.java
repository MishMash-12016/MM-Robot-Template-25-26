// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.FeedForwards;

/**
 * A helper class that computes feedforward outputs for a simple elevator (modeled as a motor acting
 * against the force of gravity).
 */
public class ElevatorFeedforward extends SimpleMotorFeedforward{

    /** The gravity gain. */
    public double kg;

    /**
     * Creates a new ElevatorFeedforward with the specified gains. Units of the gain values will
     * dictate units of the computed feedforward.
     *
     * @param ks The static gain.
     * @param kg The gravity gain.
     * @param kv The velocity gain.
     * @param ka The acceleration gain.
     * @throws IllegalArgumentException for kv &lt; zero.
     * @throws IllegalArgumentException for ka &lt; zero.
     */
    public ElevatorFeedforward(double ks, double kg, double kv, double ka) {
        super(ks, kv, ka);
        this.kg = kg;
    }

    /**
     * Creates a new ElevatorFeedforward with the specified gains. Acceleration gain is defaulted to
     * zero. Units of the gain values will dictate units of the computed feedforward.
     *
     * @param ks The static gain.
     * @param kg The gravity gain.
     * @param kv The velocity gain.
     */
    public ElevatorFeedforward(double ks, double kg, double kv) {
        this(ks, kg, kv, 0);
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param velocity The velocity setpoint.
     * @param acceleration The acceleration setpoint.
     * @return The computed feedforward.
     */
    @Override
    public double calculate(double velocity, double acceleration) {
        return ks * Math.signum(velocity) + kg + kv * velocity + ka * acceleration;
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
