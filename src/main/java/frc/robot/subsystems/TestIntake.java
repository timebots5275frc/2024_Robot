// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestIntake extends SubsystemBase {
  /** Creates a new TestIntake. */

  // private double k_pivotMotorP = 0.12;
  // private double k_pivotMotorI = 0.0;
  // private double k_pivotMotorD = 0.001;

  private CANSparkMax intakeMotor;
  private SparkPIDController intakePID;
  private RelativeEncoder intakeEncoder;



  public TestIntake() {
    
    intakeMotor = new CANSparkMax(Constants.intakeConstants.IntakeDeviceID, CANSparkLowLevel.MotorType.kBrushless);
    intakePID = intakeMotor.getPIDController();
    intakeEncoder = intakeMotor.getEncoder();
    
    intakePID.setP(Constants.PIDConstants.intakeP);
    intakePID.setI(Constants.PIDConstants.intakeI);
    intakePID.setD(Constants.PIDConstants.intakeD);
    intakePID.setFF(Constants.PIDConstants.intakeFF);
  }


  public void runIntake(double thing) {
    intakePID.setReference(1200 * thing, CANSparkBase.ControlType.kVelocity);
  }

  public void stopIntake() {
    intakePID.setReference(0, CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("IntakeSpeed", intakeEncoder.getVelocity());
  }
}
