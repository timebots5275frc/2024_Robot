// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPivotState;

public class ShooterPivotCommand extends InstantCommand {

  Shooter shooter;
  ShooterPivotState state;

  public ShooterPivotCommand(Shooter shooter, ShooterPivotState state) {
    this.shooter = shooter;
    this.state = state;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    shooter.shooterSetPivotState(state);
  }

  @Override
  public void end(boolean interrupted) {}
}
