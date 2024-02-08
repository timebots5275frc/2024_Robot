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

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  private ShooterState currentState = ShooterState.IDLE;

  private double currentPos;
  private double leftCurrentSpeed;
  private double rightCurrentSpeed;

  private double visionShooterAngle;

  public BooleanSupplier ShotNote = new BooleanSupplier() {
    public boolean getAsBoolean() { return false; };
  };

  public enum ShooterState {
    RESET,
    START,
    IDLE,
    VISION_SHOOT,
    AMP/*,
    TRAP,
    DEFAULT_SHOOT*/
    ;
    @Override
    public String toString() {
        switch(this) {
          case RESET : return "Reset";
          case START: return "Start";
          case IDLE: return "Idle";
          case VISION_SHOOT: return "Shooting with vision";
          case AMP: return "Amp";
          default: return "None";
        }
    }
  }

  public Shooter() {
    // leftShooterRunMotor = new CANSparkMax(Constants.ShooterConstants.LEFT_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    // rightShooterRunMotor = new CANSparkMax(Constants.ShooterConstants.RIGHT_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    shooterPivotMotor = new CANSparkMax(Constants.ShooterConstants.PIVOT_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    angleEncoder = new CANcoder(Constants.ShooterConstants.PIVOT_SHOOTER_CANCODER_ID);
    // leftShooterRunPID = leftShooterRunMotor.getPIDController();
    // rightShooterRunPID = rightShooterRunMotor.getPIDController();
    // leftShooterRunEncoder = leftShooterRunMotor.getEncoder();
    // rightShooterRunEncoder = rightShooterRunMotor.getEncoder();
    shooterPivotPID = shooterPivotMotor.getPIDController();
    shooterPivotEncoder = shooterPivotMotor.getEncoder();

    // leftShooterRunPID.setP(Constants.ShooterConstants.ShooterRunPIDs.P);
    // leftShooterRunPID.setI(Constants.ShooterConstants.ShooterRunPIDs.I);
    // leftShooterRunPID.setD(Constants.ShooterConstants.ShooterRunPIDs.D);
    // leftShooterRunPID.setFF(Constants.ShooterConstants.ShooterRunPIDs.kFF);

    // rightShooterRunPID.setP(Constants.ShooterConstants.ShooterRunPIDs.P);
    // rightShooterRunPID.setI(Constants.ShooterConstants.ShooterRunPIDs.I);
    // rightShooterRunPID.setD(Constants.ShooterConstants.ShooterRunPIDs.D);
    // rightShooterRunPID.setFF(Constants.ShooterConstants.ShooterRunPIDs.kFF);

    shooterPivotPID.setP(Constants.ShooterConstants.ShooterPivotPIDs.P);
    shooterPivotPID.setI(Constants.ShooterConstants.ShooterPivotPIDs.I);
    shooterPivotPID.setD(Constants.ShooterConstants.ShooterPivotPIDs.D);
    shooterPivotPID.setFF(Constants.ShooterConstants.ShooterPivotPIDs.kFF);
    shooterPivotPID.setOutputRange(-1, 1);
    shooterPivotPID.setSmartMotionMaxVelocity(Constants.ShooterConstants.SHOOTER_PIVOT_MAX_VEL, 0);
    shooterPivotPID.setSmartMotionMaxAccel(Constants.ShooterConstants.SHOOTER_PIVOT_MAX_ACCEL, 0);

    currentState = ShooterState.IDLE;
    VisionShooterCalculator.SetShooterReference(this);

  }

  public void setVisionShooterAngle() {
    visionShooterAngle = VisionShooterCalculator.GetSpeakerShooterAngle() * Constants.ShooterConstants.SHOOTER_PIVOT_ROTATIONS_PER_DEGREE;
  }

  public void shooterSetState(ShooterState state) {
    shooterPivotEncoder.setPosition(-angleEncoder.getAbsolutePosition().getValueAsDouble() * 360 * Constants.ShooterConstants.SHOOTER_PIVOT_ROTATIONS_PER_DEGREE);
    switch(state) {
      case RESET : 
      
      case START:
      shooterPivotPID.setReference(Constants.ShooterConstants.SHOOTER_START_POS, ControlType.kSmartMotion);
      // leftShooterRunPID.setReference(0, ControlType.kVelocity);
      // rightShooterRunPID.setReference(0, ControlType.kVelocity);
      break;
      case IDLE:
      shooterPivotPID.setReference(Constants.ShooterConstants.SHOOTER_IDLE_POS, ControlType.kSmartMotion);
      // leftShooterRunPID.setReference(0, ControlType.kVelocity);
      // rightShooterRunPID.setReference(0, ControlType.kVelocity);
      currentState = ShooterState.IDLE;
      break;
      case VISION_SHOOT: 
      shooterPivotPID.setReference(visionShooterAngle, ControlType.kSmartMotion);
      // leftShooterRunPID.setReference(Constants.ShooterConstants.LEFT_SHOOTER_SPEED, ControlType.kVelocity);
      // rightShooterRunPID.setReference(Constants.ShooterConstants.RIGHT_SHOOTER_SPEED, ControlType.kVelocity);
      currentState = ShooterState.VISION_SHOOT;
      break;
      case AMP:
      shooterPivotPID.setReference(Constants.ShooterConstants.SHOOTER_DEFAULT_AMP_POS, ControlType.kSmartMotion);
      // leftShooterRunPID.setReference(Constants.ShooterConstants.LEFT_AMP_SPEED, ControlType.kVelocity);
      // rightShooterRunPID.setReference(Constants.ShooterConstants.RIGHT_AMP_SPEED, ControlType.kVelocity);
      currentState = ShooterState.AMP;
      break;
      // case TRAP:
      
      // case DEFAULT_SHOOT: 
      
    }
  }

  // public boolean shooterReady() {
  //   // Change pivotready bool to work with vision value in future
  //   boolean pivotReady = ((currentPos > Constants.ShooterConstants.SHOOTER_DEFAULT_SHOOTING_POS - Constants.ShooterConstants.SHOOTER_PIVOT_ALLOWED_OFFSET)
  //     && (currentPos < Constants.ShooterConstants.SHOOTER_DEFAULT_SHOOTING_POS + Constants.ShooterConstants.SHOOTER_PIVOT_ALLOWED_OFFSET))
  //     || ((currentPos > Constants.ShooterConstants.SHOOTER_DEFAULT_AMP_POS - Constants.ShooterConstants.SHOOTER_PIVOT_ALLOWED_OFFSET)
  //     && (currentPos < Constants.ShooterConstants.SHOOTER_DEFAULT_AMP_POS + Constants.ShooterConstants.SHOOTER_PIVOT_ALLOWED_OFFSET));
  //   boolean leftReady = ((leftCurrentSpeed > Constants.ShooterConstants.LEFT_SHOOTER_SPEED - Constants.ShooterConstants.LEFT_SHOOTER_ALLOWED_OFFSET)
  //     && (leftCurrentSpeed < Constants.ShooterConstants.LEFT_SHOOTER_SPEED + Constants.ShooterConstants.LEFT_SHOOTER_ALLOWED_OFFSET))
  //     || ((leftCurrentSpeed > Constants.ShooterConstants.LEFT_AMP_SPEED - Constants.ShooterConstants.LEFT_SHOOTER_ALLOWED_OFFSET)
  //     && (leftCurrentSpeed < Constants.ShooterConstants.LEFT_AMP_SPEED + Constants.ShooterConstants.LEFT_SHOOTER_ALLOWED_OFFSET));
  //   boolean rightReady = ((rightCurrentSpeed > Constants.ShooterConstants.RIGHT_SHOOTER_SPEED - Constants.ShooterConstants.RIGHT_SHOOTER_ALLOWED_OFFSET)
  //     && (rightCurrentSpeed < Constants.ShooterConstants.RIGHT_SHOOTER_SPEED + Constants.ShooterConstants.RIGHT_SHOOTER_ALLOWED_OFFSET))
  //     || ((rightCurrentSpeed > Constants.ShooterConstants.RIGHT_AMP_SPEED- Constants.ShooterConstants.RIGHT_SHOOTER_ALLOWED_OFFSET)
  //     && (rightCurrentSpeed < Constants.ShooterConstants.RIGHT_AMP_SPEED + Constants.ShooterConstants.RIGHT_SHOOTER_ALLOWED_OFFSET));
    
  //   return (pivotReady && leftReady && rightReady);
  // }

  public double getShooterAngle() {
    return -angleEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public ShooterState getState() {
    return currentState;
  }

  @Override
  public void periodic() {
    currentPos = shooterPivotEncoder.getPosition();
    // leftCurrentSpeed = leftShooterRunEncoder.getVelocity();
    // rightCurrentSpeed = rightShooterRunEncoder.getVelocity();
    SmartDashboard.putNumber("Shooter Spark Max Pos", shooterPivotEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Pivot Vel", shooterPivotEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter CANCODER Pos", angleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    SmartDashboard.putNumber("SP Output Current", shooterPivotMotor.getOutputCurrent());

    SmartDashboard.putString("Shooter Current State", currentState.toString());
  }
}
