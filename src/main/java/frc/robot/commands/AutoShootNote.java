// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CustomTypes.ManagerCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Shooter.ShooterPivotState;
import frc.robot.subsystems.Shooter.ShooterRunState;

public class AutoShootNote extends ManagerCommand {
  Shooter shooter;
  Intake intake;
  ShooterPivotState shooterPivot;
  ShooterRunState shooterRun;

  boolean finished = false;

  public AutoShootNote(Shooter shooter, Intake intake, ShooterPivotState shooterPivot, ShooterRunState shooterRun) {
    this.shooter = shooter;
    this.intake = intake;
    this.shooterPivot = shooterPivot;
    this.shooterRun = shooterRun;
  }

  @Override
  public void initialize() {
    
    
    if (intake.limitSwitchPressed())
    {
      SequentialCommandGroup seqCommand = new SequentialCommandGroup(new ShooterPivotCommand(shooter, shooterPivot), new IntakePivotCommand(intake, IntakePivotState.IN));

      Command resetShooterCommand = new SequentialCommandGroup(new ShooterPivotCommand(shooter, ShooterPivotState.SHOOTER_NONE), new ShooterRunCommand(shooter, ShooterRunState.NONE));
      Command shootNoteCommand = new ShooterRunCommand(shooter, shooterRun).until(shooter.ShotNote).andThen(resetShooterCommand);

      seqCommand.addCommands(shootNoteCommand);

      subCommand = seqCommand;
      subCommand.schedule();
    }
    else { finished = true; }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
