// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Vision.VisionShooterCalculator;

public class Shooter extends SubsystemBase {

  private CANSparkMax leftShooterRunMotor;
  private CANSparkMax rightShooterRunMotor;
  private SparkPIDController leftShooterRunPID;
  private SparkPIDController rightShooterRunPID;
  private RelativeEncoder leftShooterRunEncoder;
  private RelativeEncoder rightShooterRunEncoder;
  private CANcoder angleEncoder;

  private CANSparkMax shooterPivotMotor;
  private SparkPIDController shooterPivotPID;
  private RelativeEncoder shooterPivotEncoder;

  private double visionShooterAngle;

  private double targetAngle;
  private double lTargetSpeed;
  private double rTargetSpeed;

  public static boolean intakeCanMove = true;
  public static boolean readyToShoot = false;

  private ShooterPivotState currentPivotState;
  private ShooterRunState currentRunState;

  public BooleanSupplier ShotNote = new BooleanSupplier() {
    public boolean getAsBoolean() { return false; };
  };

  public BooleanSupplier ShooterAtAngle = new BooleanSupplier() {
    public boolean getAsBoolean() { return readyToShoot(); }
  };

  public enum ShooterPivotState {
    START,
    AMP,
    DEFAULT_SHOOT,
    VISION_SHOOT,
    SHOOTER_NONE,
    STUPID_POS;
    //TRAP,
  }

  public enum ShooterRunState {
    NONE,
    AMP,
    SHOOT,
  }

  public Shooter() {
    leftShooterRunMotor = new CANSparkMax(Constants.ShooterConstants.LEFT_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    rightShooterRunMotor = new CANSparkMax(Constants.ShooterConstants.RIGHT_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    shooterPivotMotor = new CANSparkMax(Constants.ShooterConstants.PIVOT_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    angleEncoder = new CANcoder(Constants.ShooterConstants.PIVOT_SHOOTER_CANCODER_ID);
    leftShooterRunPID = leftShooterRunMotor.getPIDController();
    rightShooterRunPID = rightShooterRunMotor.getPIDController();
    leftShooterRunEncoder = leftShooterRunMotor.getEncoder();
    rightShooterRunEncoder = rightShooterRunMotor.getEncoder();
    shooterPivotPID = shooterPivotMotor.getPIDController();
    shooterPivotEncoder = shooterPivotMotor.getEncoder();
    shooterPivotMotor.setInverted(true);
    leftShooterRunMotor.setInverted(true);

    leftShooterRunPID.setP(Constants.ShooterConstants.ShooterRunPIDs.P);
    leftShooterRunPID.setI(Constants.ShooterConstants.ShooterRunPIDs.I);
    leftShooterRunPID.setD(Constants.ShooterConstants.ShooterRunPIDs.D);
    leftShooterRunPID.setFF(Constants.ShooterConstants.ShooterRunPIDs.kFF);

    rightShooterRunPID.setP(Constants.ShooterConstants.ShooterRunPIDs.P);
    rightShooterRunPID.setI(Constants.ShooterConstants.ShooterRunPIDs.I);
    rightShooterRunPID.setD(Constants.ShooterConstants.ShooterRunPIDs.D);
    rightShooterRunPID.setFF(Constants.ShooterConstants.ShooterRunPIDs.kFF);

    shooterPivotPID.setP(Constants.ShooterConstants.ShooterPivotPIDs.P);
    shooterPivotPID.setI(Constants.ShooterConstants.ShooterPivotPIDs.I);
    shooterPivotPID.setD(Constants.ShooterConstants.ShooterPivotPIDs.D);
    shooterPivotPID.setFF(Constants.ShooterConstants.ShooterPivotPIDs.kFF);
    shooterPivotPID.setOutputRange(-1, 1);
    shooterPivotPID.setSmartMotionMaxVelocity(Constants.ShooterConstants.SHOOTER_PIVOT_MAX_VEL, 0);
    shooterPivotPID.setSmartMotionMaxAccel(Constants.ShooterConstants.SHOOTER_PIVOT_MAX_ACCEL, 0);
    VisionShooterCalculator.SetShooterReference(this);

  }

  public void setVisionShooterAngle() {
    visionShooterAngle = VisionShooterCalculator.GetSpeakerShooterAngle() * Constants.ShooterConstants.SHOOTER_PIVOT_ROTATIONS_PER_DEGREE;
  }

  public void shooterSetPivotState(ShooterPivotState state) {
    currentPivotState = state;

    shooterPivotEncoder.setPosition(getShooterAngle() * Constants.ShooterConstants.SHOOTER_PIVOT_ROTATIONS_PER_DEGREE);
    switch(state) {
      case START:
      shooterPivotPID.setReference(Constants.ShooterConstants.SHOOTER_START_POS, ControlType.kSmartMotion);
      targetAngle = Constants.ShooterConstants.SHOOTER_START_POS;
      break;
      case VISION_SHOOT: 
      setVisionShooterAngle();
      shooterPivotPID.setReference(visionShooterAngle * Constants.ShooterConstants.SHOOTER_PIVOT_ROTATIONS_PER_DEGREE, ControlType.kSmartMotion);
      targetAngle = visionShooterAngle * Constants.ShooterConstants.SHOOTER_PIVOT_ROTATIONS_PER_DEGREE;
      break;
      case AMP:
      shooterPivotPID.setReference(Constants.ShooterConstants.SHOOTER_DEFAULT_AMP_POS, ControlType.kSmartMotion);
      targetAngle = Constants.ShooterConstants.SHOOTER_DEFAULT_AMP_POS;
      break;
      case DEFAULT_SHOOT: 
      shooterPivotPID.setReference(Constants.ShooterConstants.SHOOTER_DEFAULT_SHOOTING_POS, ControlType.kSmartMotion);
      targetAngle = Constants.ShooterConstants.SHOOTER_DEFAULT_SHOOTING_POS;
      break;
      case SHOOTER_NONE:
      shooterPivotPID.setReference(Constants.ShooterConstants.SHOOTER_NONE_POS, ControlType.kSmartMotion);
      targetAngle = Constants.ShooterConstants.SHOOTER_NONE_POS;
      break;
      case STUPID_POS: 
      shooterPivotPID.setReference(Constants.ShooterConstants.SHOOTER_STUPID_POS, ControlType.kSmartMotion);
      targetAngle = Constants.ShooterConstants.SHOOTER_STUPID_POS;
      // case TRAP:
    }
  }

    public void shooterSetRunState(ShooterRunState state) {
      currentRunState = state;

    switch(state) {
      case NONE:
      leftShooterRunPID.setReference(0, ControlType.kVelocity);
      rightShooterRunPID.setReference(0, ControlType.kVelocity);
      lTargetSpeed = 0;
      rTargetSpeed = 0;
      break;
      case AMP: 
      leftShooterRunPID.setReference(Constants.ShooterConstants.LEFT_AMP_SPEED, ControlType.kVelocity);
      rightShooterRunPID.setReference(Constants.ShooterConstants.RIGHT_AMP_SPEED, ControlType.kVelocity);
      lTargetSpeed = Constants.ShooterConstants.LEFT_AMP_SPEED;
      rTargetSpeed = Constants.ShooterConstants.RIGHT_AMP_SPEED;
      break;
      case SHOOT:
      leftShooterRunPID.setReference(Constants.ShooterConstants.LEFT_SHOOTER_SPEED, ControlType.kVelocity);
      rightShooterRunPID.setReference(Constants.ShooterConstants.RIGHT_SHOOTER_SPEED, ControlType.kVelocity);
      lTargetSpeed = Constants.ShooterConstants.LEFT_SHOOTER_SPEED;
      rTargetSpeed = Constants.ShooterConstants.RIGHT_SHOOTER_SPEED;
      break;
      // case TRAP:
    }
  }

  public boolean readyToShoot() {
    boolean pivotReached = (Constants.ShooterConstants.SHOOTER_PIVOT_ALLOWED_OFFSET > Math.abs(targetAngle - shooterPivotEncoder.getPosition()));
    boolean lSpeedReached = (lTargetSpeed > 0 && Constants.ShooterConstants.LEFT_SHOOTER_ALLOWED_DIFFERENTIAL > Math.abs(lTargetSpeed - leftShooterRunEncoder.getVelocity()));
    boolean rSpeedReached = (rTargetSpeed > 0 && Constants.ShooterConstants.RIGHT_SHOOTER_ALLOWED_DIFFERENTIAL > Math.abs(rTargetSpeed - rightShooterRunEncoder.getVelocity()));
    return pivotReached && lSpeedReached && rSpeedReached;
  }

  private boolean shooterOutOfWay() {
    return shooterPivotEncoder.getPosition() < ShooterConstants.SHOOTER_MAX_INTAKE_ANGLE;
  }

  public double getShooterAngle() {
    return angleEncoder.getAbsolutePosition().getValueAsDouble() * 360;
  }

  public double getShooterEndHeight() {
    return Math.sin(Math.toRadians(getShooterAngle())) * ShooterConstants.SHOOTER_LENGTH;
  }

  public ShooterPivotState getCurrentPivotState() { return currentPivotState; }
  public ShooterRunState getCurrentRunState() { return currentRunState; }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Spark Max Pos", shooterPivotEncoder.getPosition());
    SmartDashboard.putNumber("Shooter CANCODER Pos", angleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    SmartDashboard.putNumber("Left shooter velocity", leftShooterRunEncoder.getVelocity());
    SmartDashboard.putNumber("Right shooter vleocity", rightShooterRunEncoder.getVelocity());
    SmartDashboard.putNumber("Right shooter Current", rightShooterRunMotor.getOutputCurrent());
    SmartDashboard.putNumber("Left Shooter Current", leftShooterRunMotor.getOutputCurrent());
    Shooter.intakeCanMove = shooterOutOfWay();
    SmartDashboard.putBoolean("Shooter Out of way", intakeCanMove);
    Shooter.readyToShoot = readyToShoot();
    SmartDashboard.putBoolean("Shooter Ready", readyToShoot());
    SmartDashboard.putNumber("Target Shooter Pivot angle", VisionShooterCalculator.GetSpeakerShooterAngle());
  }
}
