// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberMode;

public class ResetClimberCommand extends Command {

  Climber climber;

  public ResetClimberCommand(Climber climber) {
    this.climber = climber;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climber.setClimberState(ClimberMode.RESET);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setClimberState(ClimberMode.NONE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}