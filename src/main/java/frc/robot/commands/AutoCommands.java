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
import frc.robot.subsystems.Vision.Vision;

/** Add your docs here. */
public class AutoCommands {

    static Vector2 leftNotePos = new Vector2(-1.35, 2.55);
    static Vector2 middleNotePos = new Vector2(0, 2.625);
    static Vector2 rightNotePos = new Vector2(1.2, 2.65);
    static Vector2 taxiPos = new Vector2(1.2, 2.85);

    static Vector2 lmTransPos = new Vector2((leftNotePos.x + middleNotePos.x) / 2 + .48, 1.75);
    static Vector2 rmTransPos = new Vector2((rightNotePos.x + middleNotePos.x) / 2, 1.75);

    static Command readyIntakeToGetNoteCommand(Intake intake)
    {
        return new SequentialCommandGroup(new IntakePivotCommand(intake, IntakePivotState.OUT), new IntakeRunCommand(intake, IntakeRunState.INTAKE));
    }

    static Command driveUntilPickedUpNoteCommand(AutoVisionDrive driveCommand, Intake intake)
    {
        return driveCommand.until(intake.NoteReadyToFeedToShooter);
    }

    public static Command leftMiddleRightAutoCommand(SwerveDrive swerveDrive, Vision vision, Shooter shooter, Intake intake)
    {
        AutoVisionDrive visionDriveNoteLeft = new AutoVisionDrive(swerveDrive, vision, leftNotePos);
        AutoVisionDrive visionDriveNoteMiddle = new AutoVisionDrive(swerveDrive, vision, middleNotePos);
        AutoVisionDrive visionDriveNoteRight = new AutoVisionDrive(swerveDrive, vision, rightNotePos);
        AutoVisionDrive visionDriveNoteMRTransition = new AutoVisionDrive(swerveDrive, vision, rmTransPos, true);
        AutoVisionDrive visionDriveNoteLMTransition = new AutoVisionDrive(swerveDrive, vision, lmTransPos, true);

        return new SequentialCommandGroup(
            new UseLimelightCommand(true),
            new WaitUntilCommand(vision.HasValidData),
            AutoVisionSpeakerShoot.ShootVisionCommandAutoFirstShot(shooter, intake, swerveDrive), 
            readyIntakeToGetNoteCommand(intake),
            new WaitCommand(.5),
            driveUntilPickedUpNoteCommand(visionDriveNoteLeft, intake),
            AutoVisionSpeakerShoot.ShootDontStopAnything(shooter, swerveDrive, vision, intake),
            readyIntakeToGetNoteCommand(intake),
            visionDriveNoteLMTransition, 
            driveUntilPickedUpNoteCommand(visionDriveNoteMiddle, intake),
            AutoVisionSpeakerShoot.ShootDontStopAnything(shooter, swerveDrive, vision, intake),
            readyIntakeToGetNoteCommand(intake),
            visionDriveNoteMRTransition,
            driveUntilPickedUpNoteCommand(visionDriveNoteRight, intake),
            AutoVisionSpeakerShoot.ShootAndStopCommand(shooter, swerveDrive, vision, intake),
            new AutoVisionDrive(swerveDrive, vision, taxiPos))
            .finallyDo((boolean interuppted) -> vision.setUsingLimelight(false));
    }

    public static Command rightMiddleLeftAutoCommand(SwerveDrive swerveDrive, Vision vision, Shooter shooter, Intake intake)
    {
        AutoVisionDrive visionDriveNoteLeft = new AutoVisionDrive(swerveDrive, vision, leftNotePos);
        AutoVisionDrive visionDriveNoteMiddle = new AutoVisionDrive(swerveDrive, vision, middleNotePos.substract(new Vector2(0.05, 0)));
        AutoVisionDrive visionDriveNoteRight = new AutoVisionDrive(swerveDrive, vision, rightNotePos);
        AutoVisionDrive visionDriveNoteMRTransition = new AutoVisionDrive(swerveDrive, vision, rmTransPos.substract(new Vector2(.48, 0)), true);
        AutoVisionDrive visionDriveNoteLMTransition = new AutoVisionDrive(swerveDrive, vision, lmTransPos, true);

        return new SequentialCommandGroup(
            new UseLimelightCommand(true),
            new WaitUntilCommand(vision.HasValidData),
            AutoVisionSpeakerShoot.ShootVisionCommandAutoFirstShot(shooter, intake, swerveDrive), 
            readyIntakeToGetNoteCommand(intake),
            new WaitCommand(.5),
            driveUntilPickedUpNoteCommand(visionDriveNoteRight, intake),
            AutoVisionSpeakerShoot.ShootDontStopAnything(shooter, swerveDrive, vision, intake),
            readyIntakeToGetNoteCommand(intake),
            visionDriveNoteMRTransition, 
            driveUntilPickedUpNoteCommand(visionDriveNoteMiddle, intake),
            AutoVisionSpeakerShoot.ShootDontStopAnything(shooter, swerveDrive, vision, intake),
            readyIntakeToGetNoteCommand(intake),
            visionDriveNoteLMTransition,
            driveUntilPickedUpNoteCommand(visionDriveNoteLeft, intake),
            AutoVisionSpeakerShoot.ShootAndStopCommand(shooter, swerveDrive, vision, intake))
            .finallyDo((boolean interuppted) -> vision.setUsingLimelight(false));
    }

    public static Command shootLeftShootAutoCommand(SwerveDrive swerveDrive, Vision vision, Shooter shooter, Intake intake)
    {
        AutoVisionDrive visionDriveNoteLeft = new AutoVisionDrive(swerveDrive, vision, leftNotePos);

        return new SequentialCommandGroup(
            new UseLimelightCommand(true),
            new WaitUntilCommand(vision.HasValidData),
            AutoVisionSpeakerShoot.ShootVisionCommandAutoFirstShot(shooter, intake, swerveDrive), 
            readyIntakeToGetNoteCommand(intake),
            new WaitCommand(.5),
            driveUntilPickedUpNoteCommand(visionDriveNoteLeft, intake),
            AutoVisionSpeakerShoot.ShootDontStopAnything(shooter, swerveDrive, vision, intake))
            .finallyDo((boolean interuppted) -> vision.setUsingLimelight(false));
    }

    public static Command shootRightShootAutoCommand(SwerveDrive swerveDrive, Vision vision, Shooter shooter, Intake intake)
    {
        AutoVisionDrive visionDriveNoteRight = new AutoVisionDrive(swerveDrive, vision, rightNotePos);

        return new SequentialCommandGroup(
            new UseLimelightCommand(true),
            new WaitUntilCommand(vision.HasValidData),
            AutoVisionSpeakerShoot.ShootVisionCommandAutoFirstShot(shooter, intake, swerveDrive), 
            readyIntakeToGetNoteCommand(intake),
            new WaitCommand(.5),
            driveUntilPickedUpNoteCommand(visionDriveNoteRight, intake),
            AutoVisionSpeakerShoot.ShootDontStopAnything(shooter, swerveDrive, vision, intake))
            .finallyDo((boolean interuppted) -> vision.setUsingLimelight(false));
    }

    public static Command shootMiddleShootAutoCommand(SwerveDrive swerveDrive, Vision vision, Shooter shooter, Intake intake)
    {
        AutoVisionDrive visionDriveNoteMiddle = new AutoVisionDrive(swerveDrive, vision, new Vector2(0, 2.625));

        return new SequentialCommandGroup(
            new UseLimelightCommand(true),
            new WaitUntilCommand(vision.HasValidData),
            AutoVisionSpeakerShoot.ShootVisionCommandAutoFirstShot(shooter, intake, swerveDrive), 
            readyIntakeToGetNoteCommand(intake),
            new WaitCommand(.5),
            driveUntilPickedUpNoteCommand(visionDriveNoteMiddle, intake),
            AutoVisionSpeakerShoot.ShootDontStopAnything(shooter, swerveDrive, vision, intake))
            .finallyDo((boolean interuppted) -> vision.setUsingLimelight(false));
    }
}