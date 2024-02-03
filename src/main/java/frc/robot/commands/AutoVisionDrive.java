// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.subsystems.DriveTrain.SwerveDrive;

import frc.robot.subsystems.Vision.Vision;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.VisionDriveCalculator;

public class AutoVisionDrive extends Command {

  static final double turnSpeed = 1.5;
  static final double driveSpeed = 3;

  SwerveDrive swerveDrive;
  Vision vision;
  Vector2 offset;

  public boolean madeItToTarget = false;

  public AutoVisionDrive(SwerveDrive swerveDrive, Vision vision, Vector2 offset) {
    this.swerveDrive = swerveDrive;
    this.vision = vision;
    this.offset = offset;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
      SmartDashboard.putString("Auto Vision Drive Target", offset.toString(3));
  }

  @Override
  public void execute()
  {
    VisionDriveCalculator.AprilTagMoveVelocity moveDirection = VisionDriveCalculator.GetVelocityToAprilTagOffset(offset);
    double rotationDirection = VisionDriveCalculator.rotateTowardsTarget();

    Vector2 moveVelocity = moveDirection.velocity.times(driveSpeed);
    double rotationVelocity = rotationDirection * turnSpeed;

    SmartDashboard.putString("AT Move velocity", moveVelocity.toString(3));
    SmartDashboard.putString("AT Rotation velocity", rotationVelocity + "");
    SmartDashboard.putString("getSubsystem()", moveDirection.distanceFromTarget + "");

    // Check if made it to target position
    if (moveDirection.validData && moveDirection.distanceFromTarget <= VisionConstants.TARGET_POSITION_ALLOWED_ERROR)
    {
      swerveDrive.drive(0, 0, 0, false);
      madeItToTarget = true;
    }
    else if (moveDirection.validData)
    {
      if (moveVelocity.magnitude() < .1) { moveVelocity = Vector2.zero; }
      swerveDrive.drive(moveVelocity.y, moveVelocity.x, rotationVelocity, false);
    }
    else { swerveDrive.drive(0, 0, 0, false); }
  }

  @Override
  public boolean isFinished() {
    return madeItToTarget;
  }
}
