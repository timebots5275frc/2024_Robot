package frc.robot.CustomTypes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public abstract class ManagerCommand extends Command {
    protected Command subCommand;
    private boolean subCommandFinished;

    @Override
    public void end(boolean interrupted)
    {
      System.out.println("ManagerCommand ended | Interrupted: " + interrupted + " | Subcommand finished: " + subCommand.isFinished());

      if (subCommand != null && !subCommand.isFinished()) 
      { 
        subCommand.cancel();
        subCommand = null;
       }
    }

    private void setSubCommandFinished()
    {
      System.out.println("Subcommand set finished");
      subCommandFinished = true;
    }

    protected void scheduleSubcommand()
    {
      if (subCommand != null)
      {
        System.out.println("Schedule Subcommand");
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
