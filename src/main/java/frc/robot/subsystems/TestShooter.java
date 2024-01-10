// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestShooter extends SubsystemBase {
  private int shooterId;
  private int shooterId2;
  private CANSparkMax shooterMotorController;
  private CANSparkMax shooterMotorController2;
  private SparkPIDController shooterMotorPID;
  private SparkPIDController shooterMotorPID2;

  /** Creates a new TestShooter. */
  public TestShooter() {
    shooterId = 21;
    shooterMotorController = new CANSparkMax(shooterId, CANSparkLowLevel.MotorType.kBrushless);
    shooterMotorPID = shooterMotorController.getPIDController();
    shooterId2 = 31;
    shooterMotorController2 = new CANSparkMax(shooterId2, CANSparkLowLevel.MotorType.kBrushless);
    shooterMotorPID2 = shooterMotorController2.getPIDController();
  }

  public void shoot() {
    shooterMotorPID.setReference(600, ControlType.kVelocity);
    shooterMotorPID2.setReference(-600, ControlType.kVelocity);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
