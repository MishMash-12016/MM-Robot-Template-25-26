// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.tuning;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class FFKsSysid extends CommandBase {
    private final Double currentRampFactor;
    private final Double minVelocity;

    private final DoubleConsumer inputConsumer;
    private final DoubleSupplier velocitySupplier;
    private final ElapsedTime timer = new ElapsedTime();
    private double currentInput = 0.0;

    public FFKsSysid(
            Double currentRampFactor,
            Double minVelocity,
            Subsystem subsystem,
            DoubleConsumer characterizationInputConsumer,
            DoubleSupplier velocitySupplier) {

        this.currentRampFactor = currentRampFactor;
        this.minVelocity = minVelocity;
        inputConsumer = characterizationInputConsumer;
        this.velocitySupplier = velocitySupplier;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        currentInput = timer.seconds() * currentRampFactor;
        inputConsumer.accept(currentInput);
    }

    @Override
    public boolean isFinished() {
        return velocitySupplier.getAsDouble() >= minVelocity;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Static Characterization output: " + currentInput);
        inputConsumer.accept(0);
    }
}
