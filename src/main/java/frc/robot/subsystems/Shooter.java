// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax leftShooterRunMotor;
  private CANSparkMax rightShooterRunMotor;
  private SparkPIDController leftShooterRunPID;
  private SparkPIDController rightShooterRunPID;
  private RelativeEncoder leftShooterRunEncoder;
  private RelativeEncoder rightShooterRunEncoder;

  private CANSparkMax shooterPivotMotor;
  private SparkPIDController shooterPivotPID;
  private RelativeEncoder shooterPivotEncoder;

  public enum ShooterState {
    REST,
    VISION_SHOOT,
    VISION_AMP,
    TRAP,
    DEFAULT_SHOOT,
    DEFAULT_AMP
  }

  public Shooter() {
    leftShooterRunMotor = new CANSparkMax(Constants.ShooterConstants.LEFT_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    rightShooterRunMotor = new CANSparkMax(Constants.ShooterConstants.RIGHT_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    shooterPivotMotor = new CANSparkMax(Constants.ShooterConstants.PIVOT_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    leftShooterRunPID = leftShooterRunMotor.getPIDController();
    rightShooterRunPID = rightShooterRunMotor.getPIDController();
    leftShooterRunEncoder = leftShooterRunMotor.getEncoder();
    rightShooterRunEncoder = rightShooterRunMotor.getEncoder();


    shooterPivotPID = shooterPivotMotor.getPIDController();
    shooterPivotEncoder = shooterPivotMotor.getEncoder();

    leftShooterRunPID.setP(Constants.ShooterConstants.ShooterRunPIDs.P);
    leftShooterRunPID.setI(Constants.ShooterConstants.ShooterRunPIDs.I);
    leftShooterRunPID.setD(Constants.ShooterConstants.ShooterRunPIDs.D);
    leftShooterRunPID.setFF(Constants.ShooterConstants.ShooterRunPIDs.kFF);

    rightShooterRunPID.setP(Constants.ShooterConstants.ShooterRunPIDs.P);
    rightShooterRunPID.setI(Constants.ShooterConstants.ShooterRunPIDs.I);
    rightShooterRunPID.setD(Constants.ShooterConstants.ShooterRunPIDs.D);
    rightShooterRunPID.setFF(Constants.ShooterConstants.ShooterRunPIDs.kFF);
  }

  public void shooterSetState(ShooterState state) {
    switch(state) {
      case REST:
      shooterPivotPID.setReference(shooterPivotEncoder.getPosition(), ControlType.kPosition);
      leftShooterRunPID.setReference(0, ControlType.kVelocity);
      rightShooterRunPID.setReference(0, ControlType.kVelocity);
      case VISION_SHOOT: 
      shooterPivotPID.setReference(shooterPivotEncoder.getPosition(), ControlType.kPosition);
      leftShooterRunPID.setReference(Constants.ShooterConstants.LEFT_SHOOTER_SPEED, ControlType.kVelocity);
      rightShooterRunPID.setReference(Constants.ShooterConstants.RIGHT_SHOOTER_SPEED, ControlType.kVelocity);

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
