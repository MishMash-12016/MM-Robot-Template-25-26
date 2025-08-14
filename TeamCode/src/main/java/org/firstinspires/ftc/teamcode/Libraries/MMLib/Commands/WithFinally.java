package org.firstinspires.ftc.teamcode.Libraries.MMLib.Commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;

public class WithFinally extends CommandBase {

  private final Command commandToRun;
  private final Runnable onEnd;

  public WithFinally(Command commandToRun, Runnable onEnd){
    this.commandToRun = commandToRun;
    this.onEnd = onEnd;
    addRequirements(commandToRun.getRequirements().toArray(new Subsystem[0]));
  }

  @Override
  public void initialize() {
    commandToRun.initialize();
  }

  @Override
  public void execute() {
    commandToRun.execute();
  }

  @Override
  public boolean isFinished() {
    return commandToRun.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    try {
      commandToRun.end(interrupted);
    } finally {
      onEnd.run();
    }
  }
}
