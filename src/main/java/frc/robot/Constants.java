// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    
  }
  public static class PIDConstants {
    public static final double intakeP = 0.0f;
    public static final double intakeI = 0.0f;
    public static final double intakeD = 0.0f;
    public static final double intakeFF= 0.001f;


    public static final double shooterP = 0.00005f;
    public static final double shooterI = 0.0f;
    public static final double shooterD = 0.0f;
    public static final double shooterFF= 0.0f;
    
  }
  public static class intakeConstants {
    public static final double pivotMotorRotationnsPer360 = 100;
    public static final double pivotMotorRotationnsPerDegree = pivotMotorRotationnsPer360 / 360;
    public static final int IntakeDeviceID = 42;
    
    public static final int ColesNEOMotor = 0;
    public static final double ColesNEOMotorPercent = 0.5f;

  }
}
