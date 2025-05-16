package org.firstinspires.ftc.teamcode.Libraries.MMLib.Commands;


import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.Subsystem;

import java.util.function.Consumer;

/**
 * this command gets a consumer, and an ON and OFF state to feed to the consumer.
 * returns a {@link StartEndCommand} where the {@link StartEndCommand#initialize() start()} method runs the ON, and the {@link StartEndCommand#end(boolean) end()} runs the OFF state.
 * @param <T> the type ur consumer gets
 */
public class MMToggleCommand<T> extends StartEndCommand {
    public MMToggleCommand(Consumer<T> set, T on, T off, Subsystem... requirements) {
        super(
                () -> set.accept(on),
                () -> set.accept(off),
                requirements
        );
    }
}
