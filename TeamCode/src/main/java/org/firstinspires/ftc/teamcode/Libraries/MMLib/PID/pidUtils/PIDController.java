// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils;

/**
 * Implements a PID control loop.
 */
public class PIDController implements AutoCloseable {

    // Factor for "proportional" control
    public double m_kp;

    // Factor for "integral" control
    public double m_ki;

    // Factor for "derivative" control
    public double m_kd;

    // The error range where "integral" control applies
    public double m_iZone = Double.POSITIVE_INFINITY;

    // The period (in seconds) of the loop that calls the controller
    public final double m_period;

    public double m_maximumIntegral = 1.0;

    public double m_minimumIntegral = -1.0;

    public double m_maximumInput;

    public double m_minimumInput;

    // Do the endpoints wrap around? e.g. Absolute encoder
    public boolean m_continuous;

    // The error at the time of the most recent call to calculate()
    public double m_error;
    public double m_errorDerivative;

    // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
    public double m_prevError;

    // The sum of the errors for use in the integral calc
    public double m_totalError;

    // The error that is considered at setpoint.
    public double m_errorTolerance = 0.05;
    public double m_errorDerivativeTolerance = Double.POSITIVE_INFINITY;

    public double m_setpoint;
    public double m_measurement;

    public boolean m_haveMeasurement;
    public boolean m_haveSetpoint;

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd and a default period of
     * 0.02 seconds.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     */
    public PIDController(double kp, double ki, double kd) {
        this(kp, ki, kd, 0.02);
    }

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd.
     *
     * @param kp     The proportional coefficient.
     * @param ki     The integral coefficient.
     * @param kd     The derivative coefficient.
     * @param period The period between controller updates in seconds.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */
    @SuppressWarnings("this-escape")
    public PIDController(double kp, double ki, double kd, double period) {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;

        if (kp < 0.0) {
            throw new IllegalArgumentException("Kp must be a non-negative number!");
        }
        if (ki < 0.0) {
            throw new IllegalArgumentException("Ki must be a non-negative number!");
        }
        if (kd < 0.0) {
            throw new IllegalArgumentException("Kd must be a non-negative number!");
        }
        if (period <= 0.0) {
            throw new IllegalArgumentException("Controller period must be a positive number!");
        }
        m_period = period;
    }

    @Override
    public void close() {
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Set the proportional, integral, and differential coefficients.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    public void setPID(double kp, double ki, double kd) {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
    }

    /**
     * Sets the Proportional coefficient of the PID controller gain.
     *
     * @param kp The proportional coefficient. Must be &gt;= 0.
     */
    public void setP(double kp) {
        m_kp = kp;
    }

    /**
     * Sets the Integral coefficient of the PID controller gain.
     *
     * @param ki The integral coefficient. Must be &gt;= 0.
     */
    public void setI(double ki) {
        m_ki = ki;
    }

    /**
     * Sets the Differential coefficient of the PID controller gain.
     *
     * @param kd The differential coefficient. Must be &gt;= 0.
     */
    public void setD(double kd) {
        m_kd = kd;
    }

    /**
     * Sets the IZone range. When the absolute value of the position error is greater than IZone, the
     * total accumulated error will reset to zero, disabling integral gain until the absolute value of
     * the position error is less than IZone. This is used to prevent integral windup. Must be
     * non-negative. Passing a value of zero will effectively disable integral gain. Passing a value
     * of {@link Double#POSITIVE_INFINITY} disables IZone functionality.
     *
     * @param iZone Maximum magnitude of error to allow integral control.
     * @throws IllegalArgumentException if iZone &lt; 0
     */
    public void setIZone(double iZone) {
        if (iZone < 0) {
            throw new IllegalArgumentException("IZone must be a non-negative number!");
        }
        m_iZone = iZone;
    }

    /**
     * Get the Proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return m_kp;
    }

    /**
     * Get the Integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return m_ki;
    }

    /**
     * Get the Differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return m_kd;
    }

    /**
     * Get the IZone range.
     *
     * @return Maximum magnitude of error to allow integral control.
     */
    public double getIZone() {
        return m_iZone;
    }

    /**
     * Returns the period of this controller.
     *
     * @return the period of the controller.
     */
    public double getPeriod() {
        return m_period;
    }

    /**
     * Returns the error tolerance of this controller. Defaults to 0.05.
     *
     * @return the error tolerance of the controller.
     */
    public double getErrorTolerance() {
        return m_errorTolerance;
    }

    /**
     * Returns the error derivative tolerance of this controller. Defaults to ∞.
     *
     * @return the error derivative tolerance of the controller.
     */
    public double getErrorDerivativeTolerance() {
        return m_errorDerivativeTolerance;
    }

    /**
     * Returns the accumulated error used in the integral calculation of this controller.
     *
     * @return The accumulated error of this controller.
     */
    public double getAccumulatedError() {
        return m_totalError;
    }

    /**
     * Sets the setpoint for the PIDController.
     *
     * @param setpoint The desired setpoint.
     */
    public void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
        m_haveSetpoint = true;

        if (m_continuous) {
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            m_error = inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
        } else {
            m_error = m_setpoint - m_measurement;
        }

        m_errorDerivative = (m_error - m_prevError) / m_period;
    }

    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return m_setpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the setpoint. The error tolerance defaults
     * to 0.05, and the error derivative tolerance defaults to ∞.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
        return m_haveMeasurement
                && m_haveSetpoint
                && Math.abs(m_error) < m_errorTolerance
                && Math.abs(m_errorDerivative) < m_errorDerivativeTolerance;
    }

    /**
     * Enables continuous input.
     *
     * <p>Rather then using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        m_continuous = true;
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
    }

    /**
     * Disables continuous input.
     */
    public void disableContinuousInput() {
        m_continuous = false;
    }

    /**
     * Returns true if continuous input is enabled.
     *
     * @return True if continuous input is enabled.
     */
    public boolean isContinuousInputEnabled() {
        return m_continuous;
    }

    /**
     * Sets the minimum and maximum contributions of the integral term.
     *
     * <p>The internal integrator is clamped so that the integral term's contribution to the output
     * stays between minimumIntegral and maximumIntegral. This prevents integral windup.
     *
     * @param minimumIntegral The minimum contribution of the integral term.
     * @param maximumIntegral The maximum contribution of the integral term.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        m_minimumIntegral = minimumIntegral;
        m_maximumIntegral = maximumIntegral;
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param errorTolerance Error which is tolerable.
     */
    public void setTolerance(double errorTolerance) {
        setTolerance(errorTolerance, Math.min(m_errorDerivativeTolerance, Double.POSITIVE_INFINITY));
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param errorTolerance           Error which is tolerable.
     * @param errorDerivativeTolerance Error derivative which is tolerable.
     */
    public void setTolerance(double errorTolerance, double errorDerivativeTolerance) {
        m_errorTolerance = errorTolerance;
        m_errorDerivativeTolerance = errorDerivativeTolerance;
    }

    /**
     * Returns the minimum contributions of the integral term.
     *
     * @return The minimum contributions of the integral term.
     */
    public double getMinimumIntegral() {
        return m_minimumIntegral;
    }

    /**
     * Returns the maximum contributions of the integral term.
     *
     * @return The maximum contributions of the integral term.
     */
    public double getMaximumIntegral() {
        return m_maximumIntegral;
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getError() {
        return m_error;
    }

    /**
     * Returns the error derivative.
     *
     * @return The error derivative.
     */
    public double getErrorDerivative() {
        return m_errorDerivative;
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     * @return The next controller output.
     */
    public double calculate(double measurement, double setpoint) {
        m_setpoint = setpoint;
        m_haveSetpoint = true;
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    public double calculate(double measurement) {
        m_measurement = measurement;
        m_prevError = m_error;
        m_haveMeasurement = true;

        if (m_continuous) {
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            m_error = inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
        } else {
            m_error = m_setpoint - m_measurement;
        }

        m_errorDerivative = (m_error - m_prevError) / m_period;

        // If the absolute value of the position error is greater than IZone, reset the total error
        if (Math.abs(m_error) > m_iZone) {
            m_totalError = 0;
        } else if (m_ki != 0) {
            m_totalError =
                    clamp(
                            m_totalError + m_error * m_period,
                            m_minimumIntegral / m_ki,
                            m_maximumIntegral / m_ki);
        }

        return m_kp * m_error + m_ki * m_totalError + m_kd * m_errorDerivative;
    }

    /**
     * Resets the previous error and the integral term.
     */
    public void reset() {
        m_error = 0;
        m_prevError = 0;
        m_totalError = 0;
        m_errorDerivative = 0;
        m_haveMeasurement = false;
    }


    /**
     * Returns modulus of input.
     *
     * @param input        Input value to wrap.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     * @return The wrapped value.
     */
    public double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low   The lower boundary to which to clamp value.
     * @param high  The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }
}
