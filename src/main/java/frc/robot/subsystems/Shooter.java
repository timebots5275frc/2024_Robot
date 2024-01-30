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

  private ShooterState currentState;

  private double currentPos;
  private double leftCurrentSpeed;
  private double rightCurrentSpeed;

  public enum ShooterState {
    START,
    IDLE,
    VISION_SHOOT,
    AMP,
    TEST/*,
    TRAP,
    DEFAULT_SHOOT*/
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
      case START:
      shooterPivotPID.setReference(Constants.ShooterConstants.SHOOTER_DEFAULT_POS, ControlType.kPosition);
      leftShooterRunPID.setReference(0, ControlType.kVelocity);
      rightShooterRunPID.setReference(0, ControlType.kVelocity);
      case IDLE:
      shooterPivotPID.setReference(shooterPivotEncoder.getPosition(), ControlType.kPosition);
      leftShooterRunPID.setReference(0, ControlType.kVelocity);
      rightShooterRunPID.setReference(0, ControlType.kVelocity);
      currentState = ShooterState.IDLE;
      case VISION_SHOOT: 
      shooterPivotPID.setReference(shooterPivotEncoder.getPosition(), ControlType.kPosition);
      leftShooterRunPID.setReference(Constants.ShooterConstants.LEFT_SHOOTER_SPEED, ControlType.kVelocity);
      rightShooterRunPID.setReference(Constants.ShooterConstants.RIGHT_SHOOTER_SPEED, ControlType.kVelocity);
      currentState = ShooterState.VISION_SHOOT;
      case AMP:
      shooterPivotPID.setReference(Constants.ShooterConstants.SHOOTER_DEFAULT_AMP_POS, ControlType.kPosition);
      leftShooterRunPID.setReference(Constants.ShooterConstants.LEFT_AMP_SPEED, ControlType.kVelocity);
      rightShooterRunPID.setReference(Constants.ShooterConstants.RIGHT_AMP_SPEED, ControlType.kVelocity);
      currentState = ShooterState.AMP;
      case TEST:
      shooterPivotPID.setReference(Constants.ShooterConstants.SHOOTER_TEST_POS, ControlType.kPosition);
      leftShooterRunPID.setReference(Constants.ShooterConstants.LEFT_SHOOTER_TEST_SPEED, ControlType.kVelocity);
      rightShooterRunPID.setReference(Constants.ShooterConstants.RIGHT_SHOOTER_TEST_SPEED, ControlType.kVelocity);
      currentState = ShooterState.TEST;
      // case TRAP:
      
      // case DEFAULT_SHOOT: 
      
    }
  }

  public boolean shooterReady() {
    // Change pivotready bool to work with vision value in future
    boolean pivotReady = ((currentPos > Constants.ShooterConstants.SHOOTER_DEFAULT_SHOOTING_POS - Constants.ShooterConstants.SHOOTER_PIVOT_ALLOWED_OFFSET)
      && (currentPos < Constants.ShooterConstants.SHOOTER_DEFAULT_SHOOTING_POS + Constants.ShooterConstants.SHOOTER_PIVOT_ALLOWED_OFFSET))
      || ((currentPos > Constants.ShooterConstants.SHOOTER_DEFAULT_AMP_POS - Constants.ShooterConstants.SHOOTER_PIVOT_ALLOWED_OFFSET)
      && (currentPos < Constants.ShooterConstants.SHOOTER_DEFAULT_AMP_POS + Constants.ShooterConstants.SHOOTER_PIVOT_ALLOWED_OFFSET));
    boolean leftReady = (leftCurrentSpeed > Constants.ShooterConstants.LEFT_SHOOTER_SPEED - Constants.ShooterConstants.LEFT_SHOOTER_ALLOWED_OFFSET)
      && (leftCurrentSpeed < Constants.ShooterConstants.LEFT_SHOOTER_SPEED + Constants.ShooterConstants.LEFT_SHOOTER_ALLOWED_OFFSET)
      || (leftCurrentSpeed > Constants.ShooterConstants.LEFT_AMP_SPEED - Constants.ShooterConstants.LEFT_SHOOTER_ALLOWED_OFFSET)
      && (leftCurrentSpeed < Constants.ShooterConstants.LEFT_AMP_SPEED + Constants.ShooterConstants.LEFT_SHOOTER_ALLOWED_OFFSET);
    boolean rightReady = (rightCurrentSpeed > Constants.ShooterConstants.RIGHT_SHOOTER_SPEED - Constants.ShooterConstants.RIGHT_SHOOTER_ALLOWED_OFFSET)
      && (rightCurrentSpeed < Constants.ShooterConstants.RIGHT_SHOOTER_SPEED + Constants.ShooterConstants.RIGHT_SHOOTER_ALLOWED_OFFSET)
      || (rightCurrentSpeed > Constants.ShooterConstants.RIGHT_AMP_SPEED- Constants.ShooterConstants.RIGHT_SHOOTER_ALLOWED_OFFSET)
      && (rightCurrentSpeed < Constants.ShooterConstants.RIGHT_AMP_SPEED + Constants.ShooterConstants.RIGHT_SHOOTER_ALLOWED_OFFSET);
    
    return (pivotReady && leftReady && rightReady);
  }

  public ShooterState getState() {
    return currentState;
  }

  @Override
  public void periodic() {
    currentPos = shooterPivotEncoder.getPosition();
    leftCurrentSpeed = leftShooterRunEncoder.getVelocity();
    rightCurrentSpeed = rightShooterRunEncoder.getVelocity();
  }
}
