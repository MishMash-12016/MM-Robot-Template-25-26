package org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils;

public class MathUtils {

    /**
     * Returns the signed smallest difference from r0 to r1, in radians, normalized to (-π, π].
     */
    public static double rotationDiff(double r0, double r1) {
        return rFullToHalf(rHalfToFull(r1) - rHalfToFull(r0));
    }

    /**
     * A true mathematical modulo for doubles: always returns a value in [0, b).
     */
    public static double mod(double a, double b) {
        double rem = a % b;
        return (rem >= 0) ? rem : (b + rem);
    }

    /**
     * Converts an angle in “half” range (−π…π] to “full” range [0…2π).
     */
    public static double rHalfToFull(double angle) {
        return mod(angle, 2 * Math.PI);
    }

    /**
     * Converts an angle in “full” range [0…2π) to “half” range (−π…π].
     */
    public static double rFullToHalf(double angle) {
        double modAngle = mod(angle, 2 * Math.PI);
        if (modAngle >= Math.PI) {
            return modAngle - 2 * Math.PI;
        } else {
            return modAngle;
        }
    }
}
