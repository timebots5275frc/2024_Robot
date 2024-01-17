// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestShooter extends SubsystemBase {
  private int shooterId;
  private int shooterId2;
  private CANSparkMax shooterMotorController;
  private CANSparkMax shooterMotorController2;
  private SparkPIDController shooterMotorPID;
  private SparkPIDController shooterMotorPID2;
  private RelativeEncoder shooterEncoder;
  private RelativeEncoder shooterEncoder2;
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
    shooterEncoder = shooterMotorController.getEncoder();
    shooterEncoder2 = shooterMotorController2.getEncoder();
    speedStick = j;
    //shooterMotorPID.setP(0.016 );
    // shooterMotorPID.setI(0.00000002);
    // shooterMotorPID.setFF(0);
    // shooterMotorPID2.setP(0.016);
    // shooterMotorPID2.setI(0.00000002);
    // shooterMotorPID2.setFF(0);

    shooterMotorPID.setP(Constants.PIDConstants.shooterP);
    shooterMotorPID.setI(Constants.PIDConstants.shooterI);
    shooterMotorPID.setD(Constants.PIDConstants.shooterD);
    shooterMotorPID.setFF(Constants.PIDConstants.shooterFF);
    shooterMotorPID2.setP(Constants.PIDConstants.shooterP);
    shooterMotorPID2.setI(Constants.PIDConstants.shooterI);
    shooterMotorPID2.setD(Constants.PIDConstants.shooterD);
    shooterMotorPID2.setFF(Constants.PIDConstants.shooterFF);
  }

  public void shoot() {
    shooterMotorPID.setReference(2100 * joyVal, CANSparkBase.ControlType.kVelocity);
    shooterMotorPID2.setReference(1050 * -joyVal, CANSparkBase.ControlType.kVelocity);
    //2100 for consistent not break
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    joyVal = (speedStick.getThrottle() - 1) / 2;
    SmartDashboard.putNumber("Spark 1", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Spark 2", shooterEncoder2.getVelocity());

  }
}
