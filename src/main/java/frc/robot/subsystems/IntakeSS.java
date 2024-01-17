// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSS extends SubsystemBase {
  private CANSparkMax csp;
  /** Creates a new IntakeSS. */
  public IntakeSS() {
    
    csp = new CANSparkMax(Constants.intakeConstants.ColesNEOMotor, CANSparkLowLevel.MotorType.kBrushless);
    csp.getPIDController().setP(Constants.PIDConstants.intakeP);
    csp.getPIDController().setI(Constants.PIDConstants.intakeI);
    csp.getPIDController().setD(Constants.PIDConstants.intakeD);
    csp.getPIDController().setFF(Constants.PIDConstants.intakeFF);
  }
  public void Pull() {
    csp.set(Constants.intakeConstants.ColesNEOMotorPercent);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
