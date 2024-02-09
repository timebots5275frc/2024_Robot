// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import java.util.concurrent.CancellationException;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeRunMotor;
  private SparkPIDController intakeRunPID;
  // private RelativeEncoder intakeRunEncoder;

  private CANSparkMax intakePivotMotor;
  private SparkPIDController intakePivotPID;
  private RelativeEncoder intakePivotEncoder;
  private CANcoder intakeAngleEncoder;

  private IntakeState currentState;

  private DigitalInput limitSwitch;


  public enum IntakeState {
    RESET,
    START,
    IDLE,
    INTAKE,
    EJECT,
    READY_TO_FEED,
    FEED_SHOOTER
  }


  public Intake(Shooter shooter) {
    intakeRunMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_RUN_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    intakeRunPID = intakeRunMotor.getPIDController();
    // intakeRunEncoder = intakeRunMotor.getEncoder();

    intakePivotMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_FLIP_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    intakePivotPID = intakePivotMotor.getPIDController();
    intakePivotEncoder = intakePivotMotor.getEncoder();
    intakeAngleEncoder = new CANcoder(0);

    intakeRunPID.setP(Constants.IntakeConstants.IntakeRunPIDs.P);
    intakeRunPID.setI(Constants.IntakeConstants.IntakeRunPIDs.I);
    intakeRunPID.setD(Constants.IntakeConstants.IntakeRunPIDs.D);
    intakeRunPID.setFF(Constants.IntakeConstants.IntakeRunPIDs.kFF);

    //Set Pivot PIDS here

    intakeSetState(IntakeState.IDLE);

    limitSwitch = new DigitalInput(0);
  }

  public void intakeSetState(IntakeState state) {
    switch(state) {
      // Moves motor to position according to rotations and starts motors
      case IDLE:
      intakePivotPID.setReference(intakePivotEncoder.getPosition(), ControlType.kPosition);
      intakeRunPID.setReference(0, ControlType.kVelocity);
      currentState = IntakeState.IDLE;
      break;
      case RESET:
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_RESET_POS, ControlType.kPosition);
      intakeRunPID.setReference(0, ControlType.kVelocity);
      currentState = IntakeState.RESET;
      case START:
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_START_POS, ControlType.kSmartMotion);
      intakeRunPID.setReference(0, ControlType.kVelocity);
      currentState = IntakeState.START;
      case INTAKE:
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_COLLECT_POS, ControlType.kPosition);
      intakeRunPID.setReference(Constants.IntakeConstants.INTAKE_RUN_SPEED, ControlType.kVelocity);
      currentState = IntakeState.INTAKE;
      break;
      case EJECT:
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_COLLECT_POS, ControlType.kPosition);
      intakeRunPID.setReference(-Constants.IntakeConstants.INTAKE_RUN_SPEED, ControlType.kVelocity);
      currentState = IntakeState.EJECT;
      break;
      case READY_TO_FEED:
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_FEED_POS, ControlType.kPosition);
      intakeRunPID.setReference(0, ControlType.kVelocity);
      currentState = IntakeState.READY_TO_FEED;
      break;
      case FEED_SHOOTER:
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_FEED_POS, ControlType.kPosition);
      intakeRunPID.setReference(-Constants.IntakeConstants.INTAKE_RUN_SPEED, ControlType.kVelocity);
      currentState = IntakeState.READY_TO_FEED;
      break;
    }
  }

  // public void feedShooter() {
  //   if (shooter.shooterReady()) {
  //     intakeSetState(IntakeState.FEED_SHOOTER);
  //   }
  // }

  public void autoReady() {
    if (currentState == IntakeState.INTAKE && limitSwitch.get()) {
      intakeSetState(IntakeState.READY_TO_FEED);
    }
  }

  public BooleanSupplier autoIntake = new BooleanSupplier() {
    public boolean getAsBoolean() {return limitSwitch.get();}
  };

  @Override
  public void periodic() {

  }
}




