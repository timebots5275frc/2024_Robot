package frc.robot.CustomTypes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public abstract class ManagerCommand extends Command {
    protected Command subCommand;
    private boolean subCommandFinished;

    @Override
    public void end(boolean interrupted)
    {
      if (subCommand != null && !subCommand.isFinished()) 
      { 
        subCommand.cancel();
        subCommand = null;
       }
    }

    private void setSubCommandFinished()
    {
      subCommandFinished = true;
    }

    protected void scheduleSubcommand()
    {
      if (subCommand != null)
      {
        subCommandFinished = false;
        subCommand.andThen(new InstantCommand(this::setSubCommandFinished)).schedule();
      }
      else { System.out.println("Trying to schedule null SubCommand!"); }
    }

    protected boolean subcommandFinished()
    {
      return subCommandFinished;
    }
}
