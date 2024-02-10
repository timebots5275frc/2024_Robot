package frc.robot.CustomTypes;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class ManagerCommand extends Command {
    protected Command subCommand;

    @Override
    public void end(boolean interrupted)
    {
      if (subCommand != null && !subCommand.isFinished()) { subCommand.cancel(); }
    }
}
