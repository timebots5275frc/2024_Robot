// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax motorL;
  private CANSparkMax motorR;
  private SparkPIDController motorLpid;
  private SparkPIDController motorRpid;
  private RelativeEncoder motorLEncoder;
  private RelativeEncoder motorREndcoder;

  private double lClimberPose;
  private double rClimberPose;

  private ClimberMode currentClimberMode;

  public Climber() {
    motorL = new CANSparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_L_ID, MotorType.kBrushless);
    motorL.setInverted(true);
    motorL.setIdleMode(IdleMode.kBrake);
    motorLpid = motorL.getPIDController();
    motorLEncoder = motorL.getEncoder();
    motorLEncoder.setPositionConversionFactor(1);
    motorLpid.setP(Constants.ClimberConstants.ClimberMotorPIDs.P, 0);
    motorLpid.setI(Constants.ClimberConstants.ClimberMotorPIDs.I, 0);
    motorLpid.setD(Constants.ClimberConstants.ClimberMotorPIDs.D, 0);
    motorLpid.setFF(Constants.ClimberConstants.ClimberMotorPIDs.kFF, 0);
    motorLpid.setIZone(Constants.ClimberConstants.ClimberMotorPIDs.IZ, 0);
    motorLpid.setP(0.008, 1);
    motorLpid.setI(0, 1);
    motorLpid.setD(0, 1);

    motorLpid.setP(Constants.ClimberConstants.ClimberMotorPIDs.P, 2);
    motorLpid.setI(Constants.ClimberConstants.ClimberMotorPIDs.I, 2);
    motorLpid.setD(Constants.ClimberConstants.ClimberMotorPIDs.D, 2);
    motorLpid.setFF(Constants.ClimberConstants.ClimberMotorPIDs.kFF * 3, 2);
    motorLpid.setIZone(Constants.ClimberConstants.ClimberMotorPIDs.IZ, 2);


    motorR = new CANSparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_R_ID, MotorType.kBrushless);
    motorRpid = motorR.getPIDController();
    motorREndcoder = motorR.getEncoder();
    motorR.setIdleMode(IdleMode.kBrake);
    motorREndcoder.setPositionConversionFactor(1);
    motorR.setInverted(false);
    motorRpid.setP(Constants.ClimberConstants.ClimberMotorPIDs.P, 0);
    motorRpid.setI(Constants.ClimberConstants.ClimberMotorPIDs.I, 0);
    motorRpid.setD(Constants.ClimberConstants.ClimberMotorPIDs.D, 0);
    motorRpid.setFF(Constants.ClimberConstants.ClimberMotorPIDs.kFF, 0);
    motorRpid.setIZone(Constants.ClimberConstants.ClimberMotorPIDs.IZ, 0);
    motorRpid.setP(0.008, 1);
    motorRpid.setI(0, 1);
    motorRpid.setD(0, 1);

    motorRpid.setP(Constants.ClimberConstants.ClimberMotorPIDs.P, 2);
    motorRpid.setI(Constants.ClimberConstants.ClimberMotorPIDs.I, 2);
    motorRpid.setD(Constants.ClimberConstants.ClimberMotorPIDs.D, 2);
    motorRpid.setFF(Constants.ClimberConstants.ClimberMotorPIDs.kFF * 3, 2);
    motorRpid.setIZone(Constants.ClimberConstants.ClimberMotorPIDs.IZ, 2);

  }
  public enum ClimberMode {
    NONE,
    EXTEND,
    RETRACT,
    RESET
  };

  public void setClimberState(ClimberMode mode) {
    currentClimberMode = mode;
    switch(mode) {
      case NONE: 
      motorLpid.setReference(0, ControlType.kVelocity, 0);
      motorRpid.setReference(0, ControlType.kVelocity, 0);
      break;
      case EXTEND: 
      if (lClimberPose < ClimberConstants.CLIMBER_MAX_POS && rClimberPose < ClimberConstants.CLIMBER_MAX_POS) {
        motorLpid.setReference(Constants.ClimberConstants.CLIMBER_SPEED, ControlType.kVelocity, 0);
        motorRpid.setReference(Constants.ClimberConstants.CLIMBER_SPEED, ControlType.kVelocity, 0);
      } else {
        setClimberState(ClimberMode.NONE);
      }
      break;
      case RETRACT:
      if (lClimberPose > 10 && rClimberPose > 10) {
        motorLpid.setReference(-Constants.ClimberConstants.CLIMBER_SPEED * 0.5, ControlType.kVelocity, 2);
        motorRpid.setReference(-Constants.ClimberConstants.CLIMBER_SPEED * 0.5, ControlType.kVelocity, 2);
      } else {
        setClimberState(ClimberMode.NONE);
      }
      break;
      case RESET:
      motorLpid.setReference(-8, ControlType.kCurrent, 1);
      motorRpid.setReference(-8, ControlType.kCurrent, 1);
      motorLEncoder.setPosition(0);
      motorREndcoder.setPosition(0);
      break;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lClimberPose = motorLEncoder.getPosition();
    rClimberPose = motorREndcoder.getPosition();
    SmartDashboard.putNumber("Left Climber height", lClimberPose);
    SmartDashboard.putNumber("Right climber height", rClimberPose);
    SmartDashboard.putNumber("Climber output current", motorL.getOutputCurrent());
    SmartDashboard.putNumber("Climber Left vel", motorLEncoder.getVelocity());
    SmartDashboard.putNumber("Climber right vel", motorREndcoder.getVelocity());

  }

  public ClimberMode climberMode() { return currentClimberMode; }
  public double leftClimberRotations() { return lClimberPose; }
  public double rightClimberRotations() { return rClimberPose; }
}
