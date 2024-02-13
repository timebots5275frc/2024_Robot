// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.CustomTypes.ManagerCommand;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.CustomTypes.Math.Vector3;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Shooter.ShooterPivotState;
import frc.robot.subsystems.Shooter.ShooterRunState;
import frc.robot.subsystems.Vision.Vision;

public class AutoVisionSpeakerShoot extends ManagerCommand {
  SwerveDrive swerveDrive;
  Shooter shooter;
  Vision vision;

  boolean finished = false;

  public AutoVisionSpeakerShoot(SwerveDrive swerveDrive, Shooter shooter, Vision vision) {
    this.swerveDrive = swerveDrive;
    this.shooter = shooter;
    this.vision = vision;
  }

  @Override
  public void initialize() 
  {
    if (vision.hasValidData())
    {
      Vector3 aprilTagPosInTargetSpace = vision.AprilTagPosInRobotSpace();
      Vector2 horizontalAprilTagPosition = new Vector2(aprilTagPosInTargetSpace.x, aprilTagPosInTargetSpace.z);
      double aprilTagDistance = horizontalAprilTagPosition.magnitude();

      if (aprilTagDistance > ShooterConstants.SPEAKER_MAX_SHOT_DISTANCE || aprilTagDistance < ShooterConstants.SPEAKER_MIN_SHOT_DISTANCE)
      {
        Vector2 targetPosRelativeToAprilTag = Vector2.clampMagnitude(horizontalAprilTagPosition, ShooterConstants.SPEAKER_MIN_SHOT_DISTANCE, ShooterConstants.SPEAKER_MAX_SHOT_DISTANCE);

        Command driveToPointInBoundsCommand = new AutoVisionDrive(swerveDrive, vision, targetPosRelativeToAprilTag);
        Command setShooterAngleCommand = new ParallelCommandGroup(new ShooterRunCommand(shooter, ShooterRunState.SHOOT), new ShooterPivotCommand(shooter, ShooterPivotState.VISION_SHOOT)).until(shooter.ShotNote);
        subCommand = new SequentialCommandGroup(driveToPointInBoundsCommand, setShooterAngleCommand);
      }
      else { subCommand = new ParallelCommandGroup(new ShooterRunCommand(shooter, ShooterRunState.SHOOT), new ShooterPivotCommand(shooter, ShooterPivotState.VISION_SHOOT)).until(shooter.ShotNote); }

      subCommand.schedule();
    }
    else { finished = true; }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
