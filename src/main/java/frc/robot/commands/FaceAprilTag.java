// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.VisionDriveCalculator;

public class FaceAprilTag extends Command {
  private SwerveDrive swerveDrive;

  boolean facingTarget = false;

  private static double turnSpeed = 1.5;

  public FaceAprilTag(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize()
  {
    facingTarget = false;
  }

  @Override
  public void execute()
  {
    double rotationDirection = VisionDriveCalculator.rotateTowardsTarget(VisionDriveCalculator.getAngleOffsetForVision());

    SmartDashboard.putNumber("Rot Dir", rotationDirection);
    if (Math.abs(rotationDirection) <= .05) { facingTarget = true; }
    else { swerveDrive.drive(0, 0, rotationDirection * turnSpeed, false); }
  }

  @Override
  public void end(boolean a)
  {
    swerveDrive.drive(00, 00000, 0.0, false);
  }

  @Override
  public boolean isFinished() {
    return facingTarget;
  }
}
