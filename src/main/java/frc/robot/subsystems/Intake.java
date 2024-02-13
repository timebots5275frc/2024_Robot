// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import java.util.concurrent.CancellationException;
import java.util.function.BooleanSupplier;

import javax.swing.JComboBox.KeySelectionManager;

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

  private IntakePivotState currentPivotState;

  private DigitalInput limitSwitch;

  public enum IntakePivotState {
    NONE,
    OUT,
    IN;
  }
  
  public enum IntakeRunState {
    NONE,
    REVERSE,
    FORWARD;
  }


  public Intake(Shooter shooter) {
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

    intakePivotPID.setP(Constants.IntakeConstants.IntakePivotPIDs.P);
    intakePivotPID.setI(Constants.IntakeConstants.IntakePivotPIDs.I);
    intakePivotPID.setD(Constants.IntakeConstants.IntakePivotPIDs.D);
    intakePivotPID.setFF(Constants.IntakeConstants.IntakePivotPIDs.kFF);
    intakePivotPID.setOutputRange(-1, 1);
    intakePivotPID.setSmartMotionMaxVelocity(Constants.IntakeConstants.INTAKE_PIVOT_MAX_VEL, 0);
    intakePivotPID.setSmartMotionMaxAccel(Constants.IntakeConstants.INTAKE_PIVOT_MAX_ACCEL, 0);
    intakePivotPID.setSmartMotionMinOutputVelocity(Constants.IntakeConstants.INTAKE_PIVOT_MIN_VEL, 0);

    limitSwitch = new DigitalInput(0);
  }

  public void intakeSetPivotState(IntakePivotState state) {
    currentPivotState = state;
    intakePivotEncoder.setPosition((intakeAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360) * Constants.IntakeConstants.INTAKE_PIVOT_ROTATIONS_PER_DEGREE);
    switch(state) {
      case NONE:
      intakePivotPID.setReference(0, ControlType.kCurrent);
      case OUT:
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_OUT_POS, ControlType.kSmartMotion);
      break;
      case IN: 
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_IN_POS, ControlType.kSmartMotion);
    }
  }

  public void intakeSetRunState(IntakeRunState state) {
    intakePivotEncoder.setPosition((intakeAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360) * Constants.IntakeConstants.INTAKE_PIVOT_ROTATIONS_PER_DEGREE);
    switch(state) {
      case NONE:
      intakeRunPID.setReference(0, ControlType.kVelocity);
      break;
      case FORWARD: 
      intakeRunPID.setReference(Constants.IntakeConstants.INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;
      case REVERSE: 
      intakeRunPID.setReference(-Constants.IntakeConstants.INTAKE_RUN_SPEED, ControlType.kVelocity);
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


  // public void feedShooter() {
  //   if (shooter.shooterReady()) {
  //     intakeSetState(IntakeState.FEED_SHOOTER);
  //   }
  // }

  // public void autoReady() {
  //   if (currentState == IntakeState.INTAKE && limitSwitch.get()) {
  //     intakeSetState(IntakeState.READY_TO_FEED);
  //   }
  // }

  // public BooleanSupplier autoIntake = new BooleanSupplier() {
  //   public boolean getAsBoolean() {return limitSwitch.get();}
  // };

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake angle", intakeAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    SmartDashboard.putNumber("Intake Pivot Rotations", intakePivotEncoder.getPosition());
    SmartDashboard.putNumber("Intake output current", intakePivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake pivot velocity", intakePivotEncoder.getVelocity());
    if (targetPosReached()) {
      intakeSetPivotState(IntakePivotState.NONE);
    }
  }
}




