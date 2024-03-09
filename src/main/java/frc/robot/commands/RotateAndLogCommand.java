// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CustomTypes.ManagerCommand;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.CustomTypes.Math.Vector3;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.Vision;

public class RotateAndLogCommand extends ManagerCommand {
  Vision vision;
  SwerveDrive swerveDrive;

  boolean isFinished = false;

  public RotateAndLogCommand(Vision vision, SwerveDrive swerveDrive) {
    this.vision = vision;
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void initialize() {
    Vector3 pos = vision.AprilTagPosInRobotSpace();
    double distanceToTag = new Vector2(pos.x, pos.z).magnitude();

    Command driveCommand = new AutoVisionDrive(swerveDrive, vision, new Vector2(0, distanceToTag));
    Command rotateCommand = new FaceAprilTag(swerveDrive, vision);

    scheduleSubcommand(new SequentialCommandGroup(driveCommand, rotateCommand));
  }

  @Override
  public boolean isFinished() {
    return isFinished || subCommandFinished();
  }

  @Override
  public void onSubCommandEnd(boolean weDontCare) {
    Vector3 pos = vision.AprilTagPosInRobotSpace();
    double distanceToTag = new Vector2(pos.x, pos.z).magnitude();

    System.out.println("Distance to tag: " + distanceToTag + " | Tx: " + vision.HorizontalOffsetFromAprilTag() + " | Ry: " + vision.AprilTagRotInRobotSpace().y);
  }
}
