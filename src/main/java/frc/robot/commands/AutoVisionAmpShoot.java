// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.CustomTypes.ManagerCommand;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.CustomTypes.Math.Vector3;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPivotState;
import frc.robot.subsystems.Shooter.ShooterRunState;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Intake.IntakeRunState;
import frc.robot.subsystems.Vision.Vision;

public class AutoVisionAmpShoot extends ManagerCommand {
  SwerveDrive swerveDrive;
  Vision vision;
  Shooter shooter;
  Intake intake;

  public static Command GetCommand(SwerveDrive swerveDrive, Vision vision, Shooter shooter, Intake intake)
  {
    SequentialCommandGroup getShooterIntakeReadyCommand = new SequentialCommandGroup(new IntakePivotCommand(intake, IntakePivotState.IN), new ShooterPivotCommand(shooter, ShooterPivotState.AMP), new ShooterRunCommand(shooter, ShooterRunState.AMP));
    SequentialCommandGroup alignAndDriveCommand = new SequentialCommandGroup(new AutoVisionDrive(swerveDrive, vision, new Vector2(0, .8f)), new FaceAprilTag(swerveDrive), new AutoVisionDrive(swerveDrive, vision, new Vector2(0, .38)));
    SequentialCommandGroup shootAndStopShooterCommand = new SequentialCommandGroup(new IntakeRunCommand(intake, IntakeRunState.OUTTAKE), new WaitCommand(1), new IntakeRunCommand(intake, IntakeRunState.NONE), new ShooterRunCommand(shooter, ShooterRunState.NONE));
    return new SequentialCommandGroup(getShooterIntakeReadyCommand, alignAndDriveCommand, shootAndStopShooterCommand);
  }

  boolean finished = false;

  public AutoVisionAmpShoot(SwerveDrive swerveDrive, Vision vision, Shooter shooter, Intake intake) {
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
        Command shootNoteCommand = new AutoShootNote(shooter, intake, ShooterPivotState.AMP, ShooterRunState.AMP);
        subCommand = new SequentialCommandGroup(driveToPointInFrontOfAmpCommand, setShooterAngleCommand, driveIntoAmpCommand, shootNoteCommand);

        scheduleSubcommand();
      }
      else { finished = true; }
    }
    else { finished = true; }
  }

  @Override
  public boolean isFinished() {
    return finished || subcommandFinished();
  }
}