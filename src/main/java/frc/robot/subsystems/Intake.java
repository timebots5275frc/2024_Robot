// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeRunMotor;
  private SparkPIDController intakeRunPID;
  private RelativeEncoder intakeRunEncoder;

  private CANSparkMax intakePivotMotor;
  private SparkPIDController intakePivotPID;
  private RelativeEncoder intakePivotEncoder;


  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    PULSE,
    FEED_SHOOTER,
  }


  public Intake() {
    intakeRunMotor = new CANSparkMax(Constants.intakeConstants.IntakeDeviceID, CANSparkLowLevel.MotorType.kBrushless);
    intakeRunPID = intakeMotor.getPIDController();
    intakeRunEncoder = intakeMotor.getEncoder();
    
    intakeRunPID.setP(Constants.PIDConstants.intakeP);
    intakeRunPID.setI(Constants.PIDConstants.intakeI);
    intakeRunPID.setD(Constants.PIDConstants.intakeD);
    intakeRunPID.setFF(Constants.PIDConstants.intakeFF);

  }

  @Override
  public void periodic() {

  }
}




