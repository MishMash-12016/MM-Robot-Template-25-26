package org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils;

import static java.util.Objects.requireNonNull;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * this class represents useful methods that may help you
 */
public abstract class MMUtils {


    /**
     * this method represents f(x),
     * while f is the linear function created by the 2 points.
     * in order to calculate the slope of the line, the method requires 2 points.
     * usually it's easier to think about the starting point, and the ending point,
     * but any 2 points will work.
     *
     * @param x      value
     * @param point1 any point on graph
     * @param point2 any point on graph
     * @return the value of x in the linear function (f(x))
     */
    public static double mapValuesLinear(double x, MMPoint2D point1, MMPoint2D point2) {
        double m = (point2.y - point1.y) / (point2.x - point1.x);
        return m * (x - point1.x) + point1.y;
    }

    public static double mapValuesLinear(DoubleSupplier x, MMPoint2D point1, MMPoint2D point2) {
        return mapValuesLinear(x.getAsDouble(), point1, point2);
    }

    /**
     * this method should help u map linearly a range of values from one range to another.
     * @param x input values
     * @param rangeInput range of the input
     * @param rangeOutput range of the output
     * @return the output on the graph of the linear function created.
     */
    public static double mapValuesLinearByRange(double x, MMRange rangeInput, MMRange rangeOutput) {
        return mapValuesLinear(
                x,
                new MMPoint2D(
                        rangeInput.start,
                        rangeOutput.start
                ),
                new MMPoint2D(
                        rangeInput.end,
                        rangeOutput.end
                )
        );
    }

    /**
     * linear transformation between joystick input to servo output
     * @param joystick joystick input
     * @return servo output
     */
    public static double joystickToServo(double joystick) {
        return mapValuesLinearByRange(
                joystick,
                new MMRange(-1, 1), //joystick range
                new MMRange(0, 1) //servo range
        );
    }

    /**
     * Requires that a parameter of a method not be null; prints an error message with helpful
     * debugging instructions if the parameter is null.
     *
     * @param <T>        Type of object.
     * @param obj        The parameter that must not be null.
     * @param paramName  The name of the parameter.
     * @param methodName The name of the method.
     * @return The object parameter confirmed not to be null.
     */
    public static <T> T requireNonNullParam(T obj, String paramName, String methodName) {
        return requireNonNull(
                obj,
                "Parameter "
                        + paramName
                        + " in method "
                        + methodName
                        + " was null when it"
                        + " should not have been!  Check the stacktrace to find the responsible line of code - "
                        + "usually, it is the first line of user-written code indicated in the stacktrace.  "
                        + "Make sure all objects passed to the method in question were properly initialized -"
                        + " note that this may not be obvious if it is being called under "
                        + "dynamically-changing conditions!  Please do not seek additional technical assistance"
                        + " without doing this first!"
        );
    }

    /**
     * Returns value clamped between min and max boundaries.
     *
     * @param value value to clamp
     * @param min   minimum valueq
     * @param max   maximum value
     * @return clamped value
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * If supplier != null, compares supplier.getAsDouble() to getter.getAsDouble().
     * If they differ, calls setter.accept(newValue).
     *
     * @param supplier  supplies the “debug” value
     * @param getter    a zero‐arg function returning the PID’s current value
     * @param setter    a one‐arg method that applies a new value to the PID
     */
    public static void updateIfChanged(
            DoubleSupplier supplier,
            DoubleSupplier getter,
            Consumer<Double> setter) {
        if (supplier == null) {
            return;
        }
        double newVal = supplier.getAsDouble();
        if (getter.getAsDouble() != newVal) {
            setter.accept(newVal);
        }
    }

    /**
     * Returns modulus of input.
     *
     * @param input Input value to wrap.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     * @return The wrapped value.
     */
    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }

    /** Multiply a 3x3 matrix (row-major) by a 3x1 vector. */
    public static double[] mat3x3MulVec(double[][] K, double[] v) {
        double[] out = new double[3];
        out[0] = K[0][0] * v[0] + K[0][1] * v[1] + K[0][2] * v[2];
        out[1] = K[1][0] * v[0] + K[1][1] * v[1] + K[1][2] * v[2];
        out[2] = K[2][0] * v[0] + K[2][1] * v[1] + K[2][2] * v[2];
        return out;
    }

    public static Pose2d PedroPoseToSolversPose2d(Pose pedroPose){
        return new Pose2d(pedroPose.getX(),
                pedroPose.getY(),
                new Rotation2d(pedroPose.getHeading()));
    }

    public static Pose Pose2dToPedro(Pose2d pose2d){
        return new Pose(pose2d.getX(),
                pose2d.getY(),
                pose2d.getHeading());
    }
}
