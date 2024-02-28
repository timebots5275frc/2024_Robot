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
        Vector2 leftNotePos = new Vector2(-1.55, 2.4);
        Vector2 middleNotePos = new Vector2(-0.1, 2.4);
        Vector2 rightNotePos = new Vector2(1, 2.55);

        Vector2 lmTransPos = new Vector2((leftNotePos.x + middleNotePos.x) / 2, 1.75);
        Vector2 rmTransPos = new Vector2((rightNotePos.x + middleNotePos.x) / 2, 1.75);

        AutoVisionDrive visionDriveNoteLeft = new AutoVisionDrive(swerveDrive, vision, leftNotePos);
        AutoVisionDrive visionDriveNoteMiddle = new AutoVisionDrive(swerveDrive, vision, middleNotePos);
        AutoVisionDrive visionDriveNoteRight = new AutoVisionDrive(swerveDrive, vision, rightNotePos);
        AutoVisionDrive visionDriveNoteMRTransition = new AutoVisionDrive(swerveDrive, vision, rmTransPos, true);
        AutoVisionDrive visionDriveNoteLMTransition = new AutoVisionDrive(swerveDrive, vision, lmTransPos, true);

        return new SequentialCommandGroup(
            new UseLimelightCommand(true),
            new WaitUntilCommand(vision.HasValidData),
            AutoVisionSpeakerShoot.ShootVisionCommandAutoFirstShot(shooter, intake, true, false), 
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
            new UseLimelightCommand(false));
    }
}