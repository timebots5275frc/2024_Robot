// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.VisionDriveCalculator;

public class AutoVisionRotate extends Command {
  private SwerveDrive swerveDrive;
  private double offset;

  boolean facingTarget = false;

  private static double turnSpeed = 1;

  public AutoVisionRotate(SwerveDrive swerveDrive, double offset) {
    this.swerveDrive = swerveDrive;
    this.offset = offset;

    addRequirements(swerveDrive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    double rotationDirection = VisionDriveCalculator.rotateTowardsTarget(offset);

    if (Math.abs(rotationDirection) <= .1) { facingTarget = true; }
    else { swerveDrive.drive(0, 0, rotationDirection * turnSpeed, false); }
  }

  @Override
  public void end(boolean a)
  {
    swerveDrive.drive(00, 00000, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return facingTarget;
  }
}
