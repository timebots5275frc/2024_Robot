// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterRunState;

public class ShooterRunCommand extends InstantCommand {

  Shooter shooter;
  ShooterRunState state;

  public ShooterRunCommand(Shooter shooter, ShooterRunState state) {
    this.shooter = shooter;
    this.state = state;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    shooter.shooterSetRunState(state);
  }

  @Override
  public void end(boolean interrupted) {}
}
