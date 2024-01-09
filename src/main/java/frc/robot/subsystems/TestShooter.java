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
  private CANSparkMax shotterMotorController2;
  private SparkPIDController shooterMotorPID;
  private SparkPIDController shooterMotorPID2;

  /** Creates a new TestShooter. */
  public TestShooter() {
    shooterId = 20;
    shooterMotorController = new CANSparkMax(shooterId, CANSparkLowLevel.MotorType.kBrushless);
    shooterMotorPID = shooterMotorController.getPIDController();
    shooterId2 = 0;
    shooterMotorController = new CANSparkMax(shooterId2, CANSparkLowLevel.MotorType.kBrushless);
    shooterMotorPID2 = shotterMotorController2.getPIDController();
  }

  public void shoot() {
    shooterMotorPID.setReference(0.25, ControlType.kCurrent);
    shooterMotorPID2.setReference(-0.25, ControlType.kCurrent);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
