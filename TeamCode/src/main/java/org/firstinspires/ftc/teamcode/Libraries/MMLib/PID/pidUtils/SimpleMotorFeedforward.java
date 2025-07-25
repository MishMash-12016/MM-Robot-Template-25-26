// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils;


/** A helper class that computes feedforward outputs for a simple permanent-magnet DC motor. */
public class SimpleMotorFeedforward{
  /** The static gain, in volts. */
  private double ks;

  /** The velocity gain, in V/(units/s). */
  private double kv;

  /** The acceleration gain, in V/(units/s²). */
  private double ka;

  /** The period, in seconds. */
  private final double m_dt;

  /**
   * Creates a new SimpleMotorFeedforward with the specified gains and period.
   *
   * <p>The units should be radians for angular systems and meters for linear systems.
   *
   * @param ks The static gain in volts.
   * @param kv The velocity gain in V/(units/s).
   * @param ka The acceleration gain in V/(units/s²).
   * @param dtSeconds The period in seconds.
   * @throws IllegalArgumentException for kv &lt; zero.
   * @throws IllegalArgumentException for ka &lt; zero.
   * @throws IllegalArgumentException for period &le; zero.
   */
  public SimpleMotorFeedforward(double ks, double kv, double ka, double dtSeconds) {
    this.ks = ks;
    this.kv = kv;
    this.ka = ka;
    if (kv < 0.0) {
      throw new IllegalArgumentException("kv must be a non-negative number, got " + kv + "!");
    }
    if (ka < 0.0) {
      throw new IllegalArgumentException("ka must be a non-negative number, got " + ka + "!");
    }
    if (dtSeconds <= 0.0) {
      throw new IllegalArgumentException(
          "period must be a positive number, got " + dtSeconds + "!");
    }
    m_dt = dtSeconds;
  }

  /**
   * Creates a new SimpleMotorFeedforward with the specified gains and period. The period is
   * defaulted to 20 ms.
   *
   * <p>The units should be radians for angular systems and meters for linear systems.
   *
   * @param ks The static gain in volts.
   * @param kv The velocity gain in V/(units/s).
   * @param ka The acceleration gain in V/(units/s²).
   * @throws IllegalArgumentException for kv &lt; zero.
   * @throws IllegalArgumentException for ka &lt; zero.
   */
  public SimpleMotorFeedforward(double ks, double kv, double ka) {
    this(ks, kv, ka, 0.020);
  }

  /**
   * Creates a new SimpleMotorFeedforward with the specified gains. Acceleration gain is defaulted
   * to zero. The period is defaulted to 20 ms.
   *
   * <p>The units should be radians for angular systems and meters for linear systems.
   *
   * @param ks The static gain in volts.
   * @param kv The velocity gain in V/(units/s).
   * @throws IllegalArgumentException for kv &lt; zero.
   */
  public SimpleMotorFeedforward(double ks, double kv) {
    this(ks, kv, 0);
  }

  /**
   * Sets the static gain.
   *
   * @param ks The static gain in volts.
   */
  public void setKs(double ks) {
    this.ks = ks;
  }

  /**
   * Sets the velocity gain.
   *
   * @param kv The velocity gain in V/(units/s).
   */
  public void setKv(double kv) {
    this.kv = kv;
  }

  /**
   * Sets the acceleration gain.
   *
   * @param ka The acceleration gain in V/(units/s²).
   */
  public void setKa(double ka) {
    this.ka = ka;
  }

  /**
   * Returns the static gain in volts.
   *
   * @return The static gain in volts.
   */
  public double getKs() {
    return ks;
  }

  /**
   * Returns the velocity gain in V/(units/s).
   *
   * <p>The units should be radians for angular systems and meters for linear systems.
   *
   * @return The velocity gain in V/(units/s).
   */
  public double getKv() {
    return kv;
  }

  /**
   * Returns the acceleration gain in V/(units/s²).
   *
   * <p>The units should be radians for angular systems and meters for linear systems.
   *
   * @return The acceleration gain in V/(units/s²).
   */
  public double getKa() {
    return ka;
  }

  /**
   * Returns the period in seconds.
   *
   * @return The period in seconds.
   */
  public double getDt() {
    return m_dt;
  }

  /**
   * Calculates the feedforward from the gains and setpoints assuming continuous control.
   *
   * @param velocity The velocity setpoint.
   * @param acceleration The acceleration setpoint.
   * @return The computed feedforward.
   * @deprecated Use {@link #calculateWithVelocities(double, double)} instead.
   */
  @SuppressWarnings("removal")
  public double calculate(double velocity, double acceleration) {
    return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
  }

  /**
   * Calculates the feedforward from the gains and velocity setpoint assuming continuous control
   * (acceleration is assumed to be zero).
   *
   * @param velocity The velocity setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double velocity) {
    return calculate(velocity, 0);
  }

  /**
   * Calculates the feedforward from the gains and setpoints assuming discrete control.
   *
   * <p>Note this method is inaccurate when the velocity crosses 0.
   *
   * @param currentVelocity The current velocity setpoint.
   * @param nextVelocity The next velocity setpoint.
   * @return The computed feedforward.
   */
  public double calculateWithVelocities(double currentVelocity, double nextVelocity) {
    // See wpimath/algorithms.md#Simple_motor_feedforward for derivation
    if (ka < 1e-9) {
      return ks * Math.signum(nextVelocity) + kv * nextVelocity;
    } else {
      double A = -kv / ka;
      double B = 1.0 / ka;
      double A_d = Math.exp(A * m_dt);
      double B_d = A > -1e-9 ? B * m_dt : 1.0 / A * (A_d - 1.0) * B;
      return ks * Math.signum(currentVelocity) + 1.0 / B_d * (nextVelocity - A_d * currentVelocity);
    }
  }

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply and an acceleration.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the acceleration constraint, and this will give you a
   * simultaneously-achievable velocity constraint.
   *
   * <p>The units should be radians for angular systems and meters for linear systems.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor, in volts.
   * @param acceleration The acceleration of the motor, in (units/s²).
   * @return The maximum possible velocity in (units/s) at the given acceleration.
   */
  public double maxAchievableVelocity(double maxVoltage, double acceleration) {
    // Assume max velocity is positive
    return (maxVoltage - ks - acceleration * ka) / kv;
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply and an acceleration.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the acceleration constraint, and this will give you a
   * simultaneously-achievable velocity constraint.
   *
   * <p>The units should be radians for angular systems and meters for linear systems.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor, in volts.
   * @param acceleration The acceleration of the motor, in (units/s²).
   * @return The maximum possible velocity in (units/s) at the given acceleration.
   */
  public double minAchievableVelocity(double maxVoltage, double acceleration) {
    // Assume min velocity is negative, ks flips sign
    return (-maxVoltage + ks - acceleration * ka) / kv;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage supply and a velocity.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * <p>The units should be radians for angular systems and meters for linear systems.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor, in volts.
   * @param velocity The velocity of the motor, in (units/s).
   * @return The maximum possible acceleration in (units/s²) at the given velocity.
   */
  public double maxAchievableAcceleration(double maxVoltage, double velocity) {
    return (maxVoltage - ks * Math.signum(velocity) - velocity * kv) / ka;
  }

  /**
   * Calculates the minimum achievable acceleration given a maximum voltage supply and a velocity.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * <p>The units should be radians for angular systems and meters for linear systems.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor, in volts.
   * @param velocity The velocity of the motor, in (units/s).
   * @return The maximum possible acceleration in (units/s²) at the given velocity.
   */
  public double minAchievableAcceleration(double maxVoltage, double velocity) {
    return maxAchievableAcceleration(-maxVoltage, velocity);
  }
}
