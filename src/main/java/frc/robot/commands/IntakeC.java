// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSS;

public class IntakeC extends Command {
  /** Creates a new IntakeC. */
  private IntakeSS inSS;
  private Joystick joyR;
  public IntakeC(IntakeSS intakeSS, Joystick joystickR) {
    this.inSS = intakeSS;
    joyR=joystickR;
    addRequirements();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joyR.getRawButtonPressed(2))   inSS.Pull();
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
