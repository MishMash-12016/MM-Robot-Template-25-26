package com.seattlesolvers.solverslib.command;

import java.util.Collections;
import java.util.Set;

class NoRequirementsCommand extends CommandBase {
  Command m_command;
  public NoRequirementsCommand(Command command){
    this.m_command = command;
  }

  @Override
  public void initialize() {
    m_command.initialize();
  }

  @Override
  public void execute() {
    m_command.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }

  @Override
  public Set<Subsystem> getRequirements() {
    // Explicitly state no requirements so scheduler doesn't bind this wrapper to any subsystem
    return Collections.emptySet();
  }
}

