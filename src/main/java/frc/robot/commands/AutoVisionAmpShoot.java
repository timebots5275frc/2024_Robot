// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.CustomTypes.ManagerCommand;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.CustomTypes.Math.Vector3;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPivotState;
import frc.robot.subsystems.Shooter.ShooterRunState;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.Vision;

public class AutoVisionAmpShoot extends ManagerCommand {
  SwerveDrive swerveDrive;
  Vision vision;
  Shooter shooter;

  boolean finished = false;

  public AutoVisionAmpShoot(SwerveDrive swerveDrive, Vision vision, Shooter shooter) {
    this.swerveDrive = swerveDrive;
    this.vision = vision;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    if (vision.hasValidData())
    {
      Vector3 aprilTagPosInTargetSpace = vision.AprilTagPosInRobotSpace();
      Vector2 horizontalAprilTagPosition = new Vector2(aprilTagPosInTargetSpace.x, aprilTagPosInTargetSpace.z);
      double aprilTagDistance = horizontalAprilTagPosition.magnitude();

      if (aprilTagDistance < VisionConstants.MAX_AMP_TARGET_DISTANCE)
      {
        Command driveToPointInFrontOfAmpCommand = new AutoVisionDrive(swerveDrive, vision, VisionConstants.AMP_VISION_DRIVE_TARGET);

        Vector2 driveIntoAmpDistance = VisionConstants.AMP_VISION_DRIVE_TARGET;
        driveIntoAmpDistance.y -= .1;
        Command driveIntoAmpCommand = new AutoOdometryDrive(swerveDrive, driveIntoAmpDistance, .5);

        Command setShooterAngleCommand = new ShooterPivotCommand(shooter, ShooterPivotState.AMP);
        Command shootNoteCommand = new ShooterRunCommand(shooter, ShooterRunState.AMP);
        subCommand = new SequentialCommandGroup(driveToPointInFrontOfAmpCommand, setShooterAngleCommand, driveIntoAmpCommand, shootNoteCommand);
      }
      else { finished = true; }
    }
    else { finished = true; }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
