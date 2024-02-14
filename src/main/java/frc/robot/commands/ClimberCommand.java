// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  private Climber cl;
  private Joystick joyL;
  public ClimberCommand(Climber _cl, Joystick _joyL) {
    cl = _cl;
    joyL = _joyL;
    addRequirements(cl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joyL.getRawButton(9)) {
      cl.extend();
    } else if (joyL.getRawButton(10)) {
      cl.retract();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
