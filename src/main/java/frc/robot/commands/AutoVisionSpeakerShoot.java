// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.CustomTypes.ManagerCommand;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.CustomTypes.Math.Vector3;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Intake.IntakeRunState;
import frc.robot.subsystems.Shooter.ShooterPivotState;
import frc.robot.subsystems.Shooter.ShooterRunState;
import frc.robot.subsystems.Vision.Vision;

public class AutoVisionSpeakerShoot extends ManagerCommand {
  SwerveDrive swerveDrive;
  Shooter shooter;
  Vision vision;
  Intake intake;

  boolean finished = false;

  public AutoVisionSpeakerShoot(SwerveDrive swerveDrive, Shooter shooter, Vision vision, Intake intake) {
    this.swerveDrive = swerveDrive;
    this.shooter = shooter;
    this.vision = vision;
    this.intake = intake;
  }

  @Override
  public void initialize() 
  {
    finished = false;

    if (vision.hasValidData())
    {
      Vector3 aprilTagPosInTargetSpace = vision.AprilTagPosInRobotSpace();
      Vector2 horizontalAprilTagPosition = new Vector2(aprilTagPosInTargetSpace.x, aprilTagPosInTargetSpace.z);
      double aprilTagDistance = horizontalAprilTagPosition.magnitude();

      Command shootNoteCommand = new SequentialCommandGroup(new ShooterPivotCommand(shooter, ShooterPivotState.VISION_SHOOT), new ShooterRunCommand(shooter, ShooterRunState.SHOOT), new WaitUntilCommand(shooter.ReadyToShoot), new IntakeRunCommand(intake, IntakeRunState.FORWARD), new WaitCommand(1), new ShooterRunCommand(shooter, ShooterRunState.NONE), new IntakeRunCommand(intake, IntakeRunState.NONE));
      Command rotateTowardsAprilTagCommand = new AutoVisionRotate(swerveDrive, 3);
      if ((aprilTagDistance > ShooterConstants.SPEAKER_MAX_SHOT_DISTANCE || aprilTagDistance < ShooterConstants.SPEAKER_MIN_SHOT_DISTANCE) && false)
      {
        Vector2 targetPosRelativeToAprilTag = Vector2.clampMagnitude(horizontalAprilTagPosition, ShooterConstants.SPEAKER_MIN_SHOT_DISTANCE, ShooterConstants.SPEAKER_MAX_SHOT_DISTANCE);

        Command driveToPointInBoundsCommand = new AutoVisionDrive(swerveDrive, vision, targetPosRelativeToAprilTag);
        subCommand = new SequentialCommandGroup(driveToPointInBoundsCommand, rotateTowardsAprilTagCommand, shootNoteCommand);
      }
      else { subCommand = new SequentialCommandGroup(rotateTowardsAprilTagCommand, shootNoteCommand); }

      scheduleSubcommand();
    }
    else 
    { 
      System.out.println("balls");
      finished = true; 
    }
  }

  @Override
  public boolean isFinished() {
  return finished || subcommandFinished();
  }
}
