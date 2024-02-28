// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Intake.IntakeRunState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Shooter.ShooterPivotState;
import frc.robot.subsystems.Shooter.ShooterRunState;
import frc.robot.subsystems.Vision.Vision;

public class AutoVisionStageShoot extends Command {

    public static Command getCommand(Shooter shooter, Intake intake, Vision vision, SwerveDrive swerveDrive)
  {
    SequentialCommandGroup shootCommand = new SequentialCommandGroup(
      new IntakeRunCommand(intake, IntakeRunState.NONE), 
      new IntakePivotCommand(intake, IntakePivotState.IN),
      new ShooterRunCommand(shooter, ShooterRunState.SHOOT),
      new AutoVisionDrive(swerveDrive, vision, new Vector2(0, 1)),
      new ShooterPivotCommand(shooter, ShooterPivotState.DEFAULT_SHOOT),  
      new WaitUntilCommand(intake.NoteReadyToFeedToShooter),
      new WaitUntilCommand(shooter.ReadyToShoot),
      new IntakeRunCommand(intake, IntakeRunState.OUTTAKE),
      new WaitCommand(.8),
      new IntakeRunCommand(intake, IntakeRunState.NONE),
      new ShooterRunCommand(shooter, ShooterRunState.NONE));
      
      return shootCommand;
  }

  /** Creates a new AutoVisionStageShoot. */
  public AutoVisionStageShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
