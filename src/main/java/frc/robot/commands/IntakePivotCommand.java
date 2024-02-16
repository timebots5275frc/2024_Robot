// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePivotState;

public class IntakePivotCommand extends InstantCommand {

  Intake intake;
  IntakePivotState state;

  public IntakePivotCommand(Intake intake, IntakePivotState state) {
    this.intake = intake;
    this.state = state;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
    intake.intakeSetPivotState(state);
  }

  @Override
  public void end(boolean interrupted) {}
}
