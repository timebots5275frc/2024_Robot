// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
  private RelativeEncoder motorLEncoder;
  private RelativeEncoder motorREndcoder;
  public Climber() {
    motorL = new CANSparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_R_ID, MotorType.kBrushless);
    motorLpid = motorL.getPIDController();
    motorLEncoder = motorL.getEncoder();
    motorLpid.setP(Constants.ClimberConstants.ClimberMotorPIDs.P);
    motorLpid.setI(Constants.ClimberConstants.ClimberMotorPIDs.I);
    motorLpid.setD(Constants.ClimberConstants.ClimberMotorPIDs.D);
    motorLpid.setFF(Constants.ClimberConstants.ClimberMotorPIDs.kFF);
    motorLpid.setIZone(Constants.ClimberConstants.ClimberMotorPIDs.IZ);
    motorLpid.setSmartMotionMaxVelocity(Constants.ClimberConstants.CLIMBER_MAX_VEL,0);

    motorR = new CANSparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_R_ID, MotorType.kBrushless);
    motorRpid = motorR.getPIDController();
    motorREndcoder = motorR.getEncoder();
    motorRpid.setP(Constants.ClimberConstants.ClimberMotorPIDs.P);
    motorRpid.setI(Constants.ClimberConstants.ClimberMotorPIDs.I);
    motorRpid.setD(Constants.ClimberConstants.ClimberMotorPIDs.D);
    motorRpid.setFF(Constants.ClimberConstants.ClimberMotorPIDs.kFF);
    motorRpid.setIZone(Constants.ClimberConstants.ClimberMotorPIDs.IZ);
    motorRpid.setSmartMotionMaxVelocity(Constants.ClimberConstants.CLIMBER_MAX_VEL, 0);

  }
  public enum ClimberMode {
    EXTEND,
    RETRACT,
    RESET
  };

  public void setClimberState(ClimberMode mode) {
    switch(mode) {
      case EXTEND: 
      motorLpid.setReference(Constants.ClimberConstants.CLIMBER_MAX, ControlType.kSmartMotion);
      motorRpid.setReference(Constants.ClimberConstants.CLIMBER_MAX, ControlType.kSmartMotion);
      break;
      case RETRACT:
      motorLpid.setReference(Constants.ClimberConstants.CLIMBER_MIN, ControlType.kSmartMotion);
      motorRpid.setReference(Constants.ClimberConstants.CLIMBER_MIN, ControlType.kSmartMotion);
      case RESET:
      motorLpid.setReference(0, ControlType.kCurrent);
      motorRpid.setReference(0, ControlType.kCurrent);
      motorLEncoder.setPosition(0);
      motorREndcoder.setPosition(0);
    }

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
