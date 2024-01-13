// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestShooter extends SubsystemBase {
  private int shooterId;
  private int shooterId2;
  private CANSparkMax shooterMotorController;
  private CANSparkMax shooterMotorController2;
  private SparkPIDController shooterMotorPID;
  private SparkPIDController shooterMotorPID2;
  private double joyVal;
  private Joystick speedStick;

  /** Creates a new TestShooter. */
  public TestShooter(Joystick j) {
    shooterId = 21;
    shooterMotorController = new CANSparkMax(shooterId, CANSparkLowLevel.MotorType.kBrushless);
    shooterMotorPID = shooterMotorController.getPIDController();
    shooterId2 = 31;
    shooterMotorController2 = new CANSparkMax(shooterId2, CANSparkLowLevel.MotorType.kBrushless);
    shooterMotorPID2 = shooterMotorController2.getPIDController();
    speedStick = j;
    shooterMotorPID.setP(0);
    shooterMotorPID.setFF(0.02);
    shooterMotorPID2.setP(0);
    shooterMotorPID2.setFF(0.02);
  }

  public void shoot() {
    shooterMotorPID.setReference(3500 * joyVal, CANSparkBase.ControlType.kVelocity);
    shooterMotorPID2.setReference(3500 * -joyVal, CANSparkBase.ControlType.kVelocity);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    joyVal = speedStick.getThrottle();

  }
}
