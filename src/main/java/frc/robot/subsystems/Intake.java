// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeRunMotor;
  private SparkPIDController intakeRunPID;
  // private RelativeEncoder intakeRunEncoder;

  private CANSparkMax intakePivotMotor;
  private SparkPIDController intakePivotPID;
  private RelativeEncoder intakePivotEncoder;

  // private IntakeState currentState;


  public enum IntakeState {
    IDLE,
    REST,
    INTAKE,
    EJECT,
    READY_TO_FEED,
    FEED_SHOOTER,
  }


  public Intake() {
    intakeRunMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_RUN_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    intakeRunPID = intakeRunMotor.getPIDController();
    // intakeRunEncoder = intakeRunMotor.getEncoder();

    intakePivotMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_FLIP_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    intakePivotPID = intakePivotMotor.getPIDController();
    intakePivotEncoder = intakePivotMotor.getEncoder();

    intakeRunPID.setP(Constants.IntakeConstants.IntakeRunPIDs.P);
    intakeRunPID.setI(Constants.IntakeConstants.IntakeRunPIDs.I);
    intakeRunPID.setD(Constants.IntakeConstants.IntakeRunPIDs.D);
    intakeRunPID.setFF(Constants.IntakeConstants.IntakeRunPIDs.kFF);

    //Set Pivot PIDS here

    intakeSetState(IntakeState.IDLE);
  }

  public void intakeSetState(IntakeState state) {
    switch(state) {
      // Moves motor to position according to rotations and starts motors
      case IDLE:
      intakePivotPID.setReference(intakePivotEncoder.getPosition(), ControlType.kPosition);
      intakeRunPID.setReference(0, ControlType.kVelocity);
      case REST:
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_DEFAULT_POS, ControlType.kPosition);
      intakeRunPID.setReference(0, ControlType.kVelocity);
      case INTAKE:
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_COLLECT_POS, ControlType.kPosition);
      intakeRunPID.setReference(Constants.IntakeConstants.INTAKE_RUN_SPEED, ControlType.kVelocity);
      case EJECT:
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_COLLECT_POS, ControlType.kPosition);
      intakeRunPID.setReference(-Constants.IntakeConstants.INTAKE_RUN_SPEED, ControlType.kVelocity);
      case READY_TO_FEED:
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_FEED_POS, ControlType.kPosition);
      intakeRunPID.setReference(0, ControlType.kVelocity);
      case FEED_SHOOTER:
      intakePivotPID.setReference(Constants.IntakeConstants.INTAKE_FEED_POS, ControlType.kPosition);
      intakeRunPID.setReference(-Constants.IntakeConstants.INTAKE_RUN_SPEED, ControlType.kVelocity);
    }
  }

  @Override
  public void periodic() {

  }
}




