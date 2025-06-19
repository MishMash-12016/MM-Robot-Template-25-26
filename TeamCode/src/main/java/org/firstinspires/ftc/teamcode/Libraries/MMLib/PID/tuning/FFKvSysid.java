package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.tuning;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.PolynomialRegression;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import Ori.Coval.Logging.Logger.KoalaLog;
public class FFKvSysid extends CommandBase {
    private final double rampRate;       // volts per second (e.g. 0.1)
    private final double kS;             // previously measured static‐friction voltage
    private final double runDuration;    // how long to ramp (seconds)

    private final DoubleConsumer setVoltage;
    private final DoubleSupplier velocitySupplier;

    private final ElapsedTime timer = new ElapsedTime();
    private final List<Double> velocityData = new ArrayList<>();
    private final List<Double> voltageMinusKs = new ArrayList<>();

    /**
     * @param rampRate         How quickly to ramp voltage (V/sec).
     * @param kS               Static‐friction voltage, measured from a separate static test.
     * @param runDuration      Total time to run this command (e.g. 5.0 seconds).
     * @param driveSubsystem   The subsystem this command requires.
     * @param setVoltage       A DoubleConsumer that applies the desired voltage to the motor.
     * @param velocitySupplier A DoubleSupplier that returns the current (absolute) velocity.
     */
    public FFKvSysid(double rampRate, double kS, double runDuration, Subsystem driveSubsystem, DoubleConsumer setVoltage, DoubleSupplier velocitySupplier) {
        this.rampRate = rampRate;
        this.kS = kS;
        this.runDuration = runDuration;
        this.setVoltage = setVoltage;
        this.velocitySupplier = velocitySupplier;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        velocityData.clear();
        voltageMinusKs.clear();
    }

    @Override
    public void execute() {
        // Ramp voltage linearly: V_raw = t * rampRate
        double elapsed = timer.seconds();
        double rawV = elapsed * rampRate;
        setVoltage.accept(rawV);

        // Read current absolute velocity
        double v = Math.abs(velocitySupplier.getAsDouble());

        // Only record points once we're above a tiny noise floor:
        if (v > 1e-3) {
            velocityData.add(v);
            voltageMinusKs.add(rawV - kS);
        }
    }

    @Override
    public boolean isFinished() {
        // Run for the specified duration (or you could also check if rawV exceeds some max).
        return timer.seconds() >= runDuration;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void end(boolean interrupted) {
        // Stop the motor
        setVoltage.accept(0.0);

        // If we have insufficient data, bail out
        if (velocityData.isEmpty() || voltageMinusKs.isEmpty()) {
            KoalaLog.log("KV SysId: Not enough data points to perform regression.", true, true);
            return;
        }

        // Convert Lists to primitive arrays
        int n = velocityData.size();
        double[] vArray = new double[n];
        double[] vmkArray = new double[n];
        for (int i = 0; i < n; i++) {
            vArray[i] = velocityData.get(i);
            vmkArray[i] = voltageMinusKs.get(i);
        }

        // Perform a first‐order polynomial regression: vmk = β0 + β1 * v
        // Since we passed in “voltage – kS,” the true intercept should be ≈ 0,
        // and β1 = kV.
        PolynomialRegression regression = new PolynomialRegression(vArray, vmkArray, 1);

        double intercept = regression.beta(0); // should be ≈ 0 if kS was perfect
        double slope = regression.beta(1); // this is our kV

        String r2Quality;
        if(regression.R2()>0.98) {
            r2Quality = "very good";
        } else if(regression.R2()>0.97) {
            r2Quality = "good";
        } else if(regression.R2()>0.96) {
            r2Quality = "ok";
        } else {
            r2Quality = "bad";
        }

        String interceptQuality;
        if (Math.abs(intercept) <= 0.02) {
            interceptQuality = "excellent";
        } else if (Math.abs(intercept) <= 0.05) {
            interceptQuality = "good";
        } else if (Math.abs(intercept) <= 0.10) {
            interceptQuality = "marginal recheck ks";
        } else {
            interceptQuality = "poor recheck ks";
        }


        KoalaLog.log("KV SysId Results:",
                String.format("  intercept (need to be close to 0) = %.6f intercept quality = %s," +
                        "kV = %.6f,  " +
                        "R² (need to be very very close to 1) = %.4f R² quality level = %s", intercept, interceptQuality, slope, regression.R2(), r2Quality), true);

    }
}
