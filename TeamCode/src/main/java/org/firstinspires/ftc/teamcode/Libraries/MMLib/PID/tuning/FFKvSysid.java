package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.PolynomialRegression;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FFKvSysid extends CommandBase {
    private static final double START_DELAY_SECS = 2.0;
    private static final double RAMP_VOLTS_PER_SEC = 0.1;

    private FeedForwardCharacterizationData data;
    private final Consumer<Double> powerConsumer;
    private final Supplier<Double> velocitySupplier;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Creates a new FeedForwardCharacterization command.
     */
    public FFKvSysid(
            Subsystem subsystem, Consumer<Double> powerConsumer, Supplier<Double> velocitySupplier) {
        addRequirements(subsystem);
        this.powerConsumer = powerConsumer;
        this.velocitySupplier = velocitySupplier;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        data = new FeedForwardCharacterizationData();
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.seconds() < START_DELAY_SECS) {
            powerConsumer.accept(0.0);
        } else {
            double power = (timer.seconds() - START_DELAY_SECS) * RAMP_VOLTS_PER_SEC;
            powerConsumer.accept(power);
            data.add(velocitySupplier.get(), power);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        powerConsumer.accept(0.0);
        data.print();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public static class FeedForwardCharacterizationData {
        private final List<Double> velocityData = new LinkedList<>();
        private final List<Double> powerData = new LinkedList<>();

        public void add(double velocity, double power) {
            if (Math.abs(velocity) > 1E-4) {
                velocityData.add(Math.abs(velocity));
                powerData.add(Math.abs(power));
            }
        }

        public void print() {
            if (velocityData.isEmpty() || powerData.isEmpty()) {
                return;
            }

            PolynomialRegression regression =
                    new PolynomialRegression(
                            velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
                            powerData.stream().mapToDouble(Double::doubleValue).toArray(),
                            1);

            //between 0-1 how good are the results (the higher the better)
            FtcDashboard.getInstance().getTelemetry().addData("FF Characterization Results:" ,regression.R2());
            //how many data points were used (the higher the better)
            FtcDashboard.getInstance().getTelemetry().addData("amount of data points", Integer.toString(velocityData.size()));
            //the KS value of the feedforward
            FtcDashboard.getInstance().getTelemetry().addData("ks", regression.beta(1));
        }
    }
}
