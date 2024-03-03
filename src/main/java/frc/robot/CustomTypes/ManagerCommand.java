package frc.robot.CustomTypes;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class ManagerCommand extends Command {
    private Command subCommand;

    private boolean subCommandFinished;
    private boolean subCommandScheduled;
    private boolean runWhenDisabled;

    protected void scheduleSubcommand(Command newSubCommand)
    {
      if (subCommandScheduled) { System.out.println("Trying to schedule SubCommand when one is already running!"); }
      else if (newSubCommand != null)
      {
        subCommand = newSubCommand;

        subCommandFinished = false;
        subCommandScheduled = true;

        m_requirements = subCommand.getRequirements();
        runWhenDisabled = subCommand.runsWhenDisabled();

        subCommand.initialize();
      }
      else { System.out.println("Trying to schedule null SubCommand!"); }
    }

    protected boolean subCommandFinished() {
      return subCommandFinished;
    }

    protected boolean subCommandScheduled() {
      return subCommandScheduled;
    }

    @Override
    public final void execute() {
      if (!subCommandScheduled) { return; }

      subCommand.execute();
      if (subCommand.isFinished()) {
        subCommand.end(false);
        onSubCommandEnd(false);

        subCommandFinished = true;
        subCommandScheduled = false;
      }
    }

    @Override
    public final boolean runsWhenDisabled() {
      return runWhenDisabled;
    }

    @Override
    public final void end(boolean interrupted) {
      onManagerCommandEnd(interrupted);

      if (subCommand != null && !subCommand.isFinished()) { 
        subCommand.end(interrupted);
        onSubCommandEnd(interrupted);
      }
    }

    public void onSubCommandEnd(boolean interrupted) {}
    public void onManagerCommandEnd(boolean interrupted) {}
}
