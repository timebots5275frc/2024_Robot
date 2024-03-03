// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.CustomTypes.ManagerCommand;
import frc.robot.CustomTypes.SetUsingLimelightFalse;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.CustomTypes.Math.Vector3;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Intake.IntakeRunState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPivotState;
import frc.robot.subsystems.Shooter.ShooterRunState;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.Vision;

public class AutoVisionSpeakerShoot extends ManagerCommand {
  SwerveDrive swerveDrive;
  Shooter shooter;
  Vision vision;
  Intake intake;

  boolean finished = false;

  static Command ShootVisionCommand(Shooter shooter, Intake intake, boolean stopIntake, boolean stopShooter)
  {
    SequentialCommandGroup shootCommand = new SequentialCommandGroup(
      new UseLimelightCommand(true),
      new WaitUntilCommand(Vision.Instance.LookingAtSpeakerTag),
      new IntakeRunCommand(intake, IntakeRunState.NONE), 
      new IntakePivotCommand(intake, IntakePivotState.IN),
      new ShooterPivotCommand(shooter, ShooterPivotState.VISION_SHOOT), 
      new ShooterRunCommand(shooter, ShooterRunState.SHOOT), 
      new WaitUntilCommand(intake.NoteReadyToFeedToShooter),
      new WaitUntilCommand(shooter.ReadyToShoot), 
      new IntakeRunCommand(intake, IntakeRunState.OUTTAKE));

      if (stopIntake || stopShooter) { shootCommand.addCommands(new WaitCommand(.8)); }
      else { shootCommand.addCommands(new WaitCommand(.6)); }
      if (stopIntake) { shootCommand.addCommands(new IntakeRunCommand(intake, IntakeRunState.NONE)); }
      if (stopShooter) { shootCommand.addCommands(new ShooterRunCommand(shooter, ShooterRunState.NONE)); }
      
      return shootCommand.finallyDo(new SetUsingLimelightFalse());
  }

  public static Command ShootVisionCommandAutoFirstShot(Shooter shooter, Intake intake, SwerveDrive swerveDrive)
  {
    SequentialCommandGroup shootCommand = new SequentialCommandGroup(
      new UseLimelightCommand(true),
      new IntakeRunCommand(intake, IntakeRunState.NONE), 
      new IntakePivotCommand(intake, IntakePivotState.IN),
      new ShooterPivotCommand(shooter, ShooterPivotState.VISION_SHOOT), 
      new ShooterRunCommand(shooter, ShooterRunState.SHOOT), 
      new WaitUntilCommand(intake.NoteReadyToFeedToShooter),
      new UseLimelightCommand(false),
      new WaitCommand(.6),
      new IntakeRunCommand(intake, IntakeRunState.OUTTAKE),
      new WaitCommand(.8),
      new IntakeRunCommand(intake, IntakeRunState.NONE));

      return shootCommand.finallyDo(new SetUsingLimelightFalse());
  }

  public static Command ShootAndStopCommand(Shooter shooter, SwerveDrive swerveDrive, Vision vision, Intake intake) {
    Command shootNoteCommand = ShootVisionCommand(shooter, intake, true, true);
    Command rotateTowardsAprilTagCommand = new FaceAprilTag(swerveDrive);

    return new SequentialCommandGroup(rotateTowardsAprilTagCommand, shootNoteCommand);
  }

  public static Command ShootCommand(Shooter shooter, SwerveDrive swerveDrive, Vision vision, Intake intake) {
    Command shootNoteCommand = ShootVisionCommand(shooter, intake, true, false);
    Command rotateTowardsAprilTagCommand = new FaceAprilTag(swerveDrive);

    return new SequentialCommandGroup(rotateTowardsAprilTagCommand, shootNoteCommand);
  }

  public static Command ShootDontStopAnything(Shooter shooter, SwerveDrive swerveDrive, Vision vision, Intake intake) {
    Command shootNoteCommand = ShootVisionCommand(shooter, intake, false, false);
    Command rotateTowardsAprilTagCommand = new FaceAprilTag(swerveDrive);

    return new SequentialCommandGroup(rotateTowardsAprilTagCommand, shootNoteCommand);
  }

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

    vision.setUsingLimelight(true);

    if (vision.hasValidData())
    {
      Vector3 aprilTagPosInTargetSpace = vision.AprilTagPosInRobotSpace();
      Vector2 xzPlaneAprilTagPosition = new Vector2(aprilTagPosInTargetSpace.x, aprilTagPosInTargetSpace.z);
      double aprilTagDistance = xzPlaneAprilTagPosition.magnitude();

      Command shootNoteCommand = new SequentialCommandGroup(new ShooterPivotCommand(shooter, ShooterPivotState.VISION_SHOOT), new ShooterRunCommand(shooter, ShooterRunState.SHOOT), new WaitUntilCommand(shooter.ReadyToShoot), new IntakeRunCommand(intake, IntakeRunState.OUTTAKE), new WaitCommand(1), new ShooterRunCommand(shooter, ShooterRunState.NONE), new IntakeRunCommand(intake, IntakeRunState.NONE));
      Command rotateTowardsAprilTagCommand = new AutoVisionRotate(swerveDrive, 3);

      scheduleSubcommand(new SequentialCommandGroup(rotateTowardsAprilTagCommand, shootNoteCommand));
    } 
    else { finished = true; }
  }

  @Override
  public boolean isFinished() {
    return finished || subCommandFinished();
  }

  @Override
  public void onSubCommandEnd(boolean interrupted) {
    vision.setUsingLimelight(false);
  }
}