// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeRunMotor;
  private SparkPIDController intakeRunPID;
  private RelativeEncoder intakeRunEncoder;

  private CANSparkMax intakePivotMotor;
  private SparkPIDController intakePivotPID;
  private RelativeEncoder intakePivotEncoder;
  private CANcoder intakeAngleEncoder;

  private DigitalInput limitSwitch;

  private IntakePivotState currentPivotState;
  private IntakeRunState currentRunState;

  public BooleanSupplier LimitSwitchIsPressed = new BooleanSupplier() {
    public boolean getAsBoolean() { return limitSwitchPressed(); };
  };

  public BooleanSupplier LimitSwitchIsNotPressed = new BooleanSupplier() {
    public boolean getAsBoolean() { return !limitSwitchPressed(); };
  };

  public BooleanSupplier NoteReadyToFeedToShooter = new BooleanSupplier() {
    public boolean getAsBoolean() { return intakePivotEncoder.getPosition() > 58; };
  };

  public enum IntakePivotState {
    NONE,
    OUT,
    IN;

    @Override 
    public String toString() {
      switch(this) {
        case NONE : return "NONE";
        case OUT : return "OUT";
        case IN : return "IN";
        default : return "No State";
      }
    }
  }
  
  public enum IntakeRunState {
    NONE,
    INTAKE,
    OUTTAKE,
    OUTTAKE_AMP;
  }


  public Intake() {
    intakeRunMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_RUN_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    intakeRunPID = intakeRunMotor.getPIDController();
    intakeRunEncoder = intakeRunMotor.getEncoder();

    intakePivotMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_PIVOT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    intakePivotPID = intakePivotMotor.getPIDController();
    intakePivotEncoder = intakePivotMotor.getEncoder();
    intakeAngleEncoder = new CANcoder(Constants.IntakeConstants.INTAKE_ANGLE_ENCODER_ID);

    intakeRunPID.setP(Constants.IntakeConstants.IntakeRunPIDs.P);
    intakeRunPID.setI(Constants.IntakeConstants.IntakeRunPIDs.I);
    intakeRunPID.setD(Constants.IntakeConstants.IntakeRunPIDs.D);
    intakeRunPID.setFF(Constants.IntakeConstants.IntakeRunPIDs.kFF);

    intakePivotPID.setP(Constants.IntakeConstants.IntakePivotPIDs.P, 0);
    intakePivotPID.setI(Constants.IntakeConstants.IntakePivotPIDs.I, 0);
    intakePivotPID.setD(Constants.IntakeConstants.IntakePivotPIDs.D, 0);
    intakePivotPID.setFF(Constants.IntakeConstants.IntakePivotPIDs.kFF, 0);
    intakePivotPID.setOutputRange(-1, 1, 0);
    intakePivotPID.setSmartMotionMaxVelocity(Constants.IntakeConstants.INTAKE_PIVOT_MAX_VEL, 0);
    intakePivotPID.setSmartMotionMaxAccel(Constants.IntakeConstants.INTAKE_PIVOT_MAX_ACCEL, 0);
    intakePivotPID.setSmartMotionMinOutputVelocity(Constants.IntakeConstants.INTAKE_PIVOT_MIN_VEL, 0);

    currentPivotState = IntakePivotState.NONE;

    limitSwitch = new DigitalInput(9);
  }

  public void intakeSetPivotState(IntakePivotState state) {
    if (Shooter.intakeCanMove) {
      currentPivotState = state;
      intakePivotEncoder.setPosition((intakeAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360) * Constants.IntakeConstants.INTAKE_PIVOT_ROTATIONS_PER_DEGREE);
      switch(state) {
        case NONE:
        intakePivotPID.setReference(0, ControlType.kCurrent, 1);
        break;
        case OUT:
        intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_OUT_POS, ControlType.kSmartMotion);
        break;
        case IN: 
        intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_IN_POS, ControlType.kSmartMotion);
        break;
      }
    }
  }

  public void intakeSetRunState(IntakeRunState state) {
    intakePivotEncoder.setPosition((intakeAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360) * Constants.IntakeConstants.INTAKE_PIVOT_ROTATIONS_PER_DEGREE);
    switch(state) {
      case NONE:
      intakeRunPID.setReference(0, ControlType.kVelocity);
      break;
      case OUTTAKE: 
      intakeRunPID.setReference(-Constants.IntakeConstants.INTAKE_RUN_SPEED * 0.4, ControlType.kVelocity);
      System.out.println("Outtaking at shooter angle: " + Shooter.targetAngle);
      break;
      case INTAKE: 
      intakeRunPID.setReference(Constants.IntakeConstants.INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;
      case OUTTAKE_AMP:
      intakeRunPID.setReference(-Constants.IntakeConstants.INTAKE_RUN_SPEED * 0.2, ControlType.kVelocity);
      break;
    }
  }

  public boolean targetPosReached() {
    if (currentPivotState == IntakePivotState.IN && Constants.IntakeConstants.INTAKE_PIVOT_ALLOWED_OFFSET > Math.abs(intakePivotEncoder.getPosition() - Constants.IntakeConstants.INTAKE_IN_POS)) {
      return true;
    } else if (currentPivotState == IntakePivotState.OUT && Constants.IntakeConstants.INTAKE_PIVOT_ALLOWED_OFFSET > Math.abs(intakePivotEncoder.getPosition() - Constants.IntakeConstants.INTAKE_OUT_POS)) {
      return true;
    }
    return false;
  }


  // public void autoFeedShooter() {
  //   if (RobotContainer.shooterReadyToShoot && intakePivotEncoder.getPosition() > Constants.IntakeConstants.INTAKE_UP_POS) {
  //     intakeSetRunState(IntakeRunState.FORWARD);
  //   }
  // }

  public void autoFlip() {
    if (currentPivotState == IntakePivotState.NONE && limitSwitchPressed() && intakePivotEncoder.getPosition() < Constants.IntakeConstants.INTAKE_UP_POS) {
      intakeSetPivotState(IntakePivotState.IN);
      intakeSetRunState(IntakeRunState.NONE);
    }
  }

  public boolean limitSwitchPressed() {
    return limitSwitch.get();
  }

  // public BooleanSupplier autoIntake = new BooleanSupplier() {
  //   public boolean getAsBoolean() {return limitSwitch.get();}
  // };

  public IntakePivotState getCurrentPivotState() { return currentPivotState; }
  public IntakeRunState getCurrentRunState() { return currentRunState; }
  public double getCurrentPivotAngle() { return intakeAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360; }

  @Override
  public void periodic() {
    boolean targetReached = targetPosReached();
    SmartDashboard.putNumber("Intake angle", intakeAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    SmartDashboard.putNumber("Intake Pivot Rotations", intakePivotEncoder.getPosition());
    SmartDashboard.putNumber("Intake Pivot Velocity", intakePivotEncoder.getVelocity());
    SmartDashboard.putNumber("Intake run speed", intakeRunEncoder.getVelocity());
    SmartDashboard.putBoolean("Inside threshold", targetReached);
    SmartDashboard.putBoolean("Note ready to feed", NoteReadyToFeedToShooter.getAsBoolean());
    if (targetReached) {
      intakeSetPivotState(IntakePivotState.NONE);
    }
    autoFlip();
    // autoFeedShooter();
  }
}




