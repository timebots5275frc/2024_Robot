// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.CustomTypes.Math.Vector3;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Vision.Vision;

public class AutoVisionSpeakerShoot extends Command {
  SwerveDrive swerveDrive;
  Shooter shooter;
  Vision vision;

  boolean finished = false;

  public AutoVisionSpeakerShoot(SwerveDrive swerveDrive, Shooter shooter, Vision vision) {
    this.swerveDrive = swerveDrive;
    this.shooter = shooter;
    this.vision = vision;

    addRequirements(swerveDrive);
    addRequirements(shooter);
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
        AutoVisionDrive autoVisionDriveCommand = new AutoVisionDrive(swerveDrive, vision, Vector2.clampMagnitude(horizontalAprilTagPosition, ShooterConstants.SPEAKER_MIN_SHOT_DISTANCE, ShooterConstants.SPEAKER_MAX_SHOT_DISTANCE));
      }
      else
      {
        ShooterCommand shooterCommand = new ShooterCommand(shooter, ShooterState.VISION_SHOOT);
        shooterCommand.schedule();
      }
    }
    else { finished = true; }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
