// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestIntake extends SubsystemBase {
  /** Creates a new TestIntake. */

  // private double k_pivotMotorP = 0.12;
  // private double k_pivotMotorI = 0.0;
  // private double k_pivotMotorD = 0.001;

` private CANSparkMax intakeMotor;
  private SparkPIDController intakePID;
  private RelativeEncoder intakeEncoder;
  private double intakeMotorP;



  public TestIntake() {
    intakeMotor = new CANSparkMax(placeholder, CANSparkLowLevel.MotorType.kBrushless);
    intakePID = intakeMotor.getPIDController();
    intakeEncoder = intakeMotor.getEncoder();
    intakeMotorP = 0.001;
    intakePID.setP(intakeMotorP);
  }


  public void runIntake() {
    intakePID.setReference(500, CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
