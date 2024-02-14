// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax motorL;
  private CANSparkMax motorR;
  private SparkPIDController motorLpid;
  private SparkPIDController motorRpid;
  public Climber() {
    motorL = new CANSparkMax( Constants.ClimberConstants.CLIMBER_MOTOR_R_ID , 
  MotorType.kBrushless);
    motorLpid = motorL.getPIDController();
    motorLpid.setP(Constants.ClimberConstants.ClimberMotorPIDs.P);
    motorLpid.setI(Constants.ClimberConstants.ClimberMotorPIDs.I);
    motorLpid.setD(Constants.ClimberConstants.ClimberMotorPIDs.D);
    motorLpid.setFF(Constants.ClimberConstants.ClimberMotorPIDs.kFF);
    motorLpid.setIZone(Constants.ClimberConstants.ClimberMotorPIDs.IZ);
    motorLpid.setSmartMotionMaxVelocity(Constants.ClimberConstants.CLIMBER_MAX_VEL,0);

    motorR = new CANSparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_R_ID, MotorType.kBrushless);
    motorRpid = motorR.getPIDController();
    motorRpid.setP(Constants.ClimberConstants.ClimberMotorPIDs.P);
    motorRpid.setI(Constants.ClimberConstants.ClimberMotorPIDs.I);
    motorRpid.setD(Constants.ClimberConstants.ClimberMotorPIDs.D);
    motorRpid.setFF(Constants.ClimberConstants.ClimberMotorPIDs.kFF);
    motorRpid.setIZone(Constants.ClimberConstants.ClimberMotorPIDs.IZ);
    motorRpid.setSmartMotionMaxVelocity(Constants.ClimberConstants.CLIMBER_MAX_VEL, 0);

  }

  public void extend() {
    motorLpid.setReference(Constants.ClimberConstants.CLIMBER_MAX, ControlType.kSmartMotion);
    motorRpid.setReference(Constants.ClimberConstants.CLIMBER_MAX, ControlType.kSmartMotion);
  }
  public void retract() {
        motorLpid.setReference(Constants.ClimberConstants.CLIMBER_MIN, ControlType.kSmartMotion);
        motorRpid.setReference(Constants.ClimberConstants.CLIMBER_MIN, ControlType.kSmartMotion);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
