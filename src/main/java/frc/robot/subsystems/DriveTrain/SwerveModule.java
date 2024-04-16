package frc.robot.subsystems.DriveTrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.DriveConstants;

public class SwerveModule {

    public CANSparkMax driveMotor;
    private CANSparkMax steerMotor;
    public RelativeEncoder driveNEOMotorEncoder; // NEO build-in Encoder

    private CANcoder steerAngleEncoder;

    private PIDController steerAnglePID;
    private SparkPIDController steerMotorVelocityPID;
    private SparkPIDController driveMotorVelocityPID;
    private int driveMotorID;

    public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderId) {

        this.driveMotorID = driveMotorID;

        driveMotor = new CANSparkMax(this.driveMotorID, CANSparkLowLevel.MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorID, CANSparkLowLevel.MotorType.kBrushless);

        steerAngleEncoder = new CANcoder(steerEncoderId);

        driveNEOMotorEncoder = driveMotor.getEncoder();
        driveNEOMotorEncoder.setPositionConversionFactor(DriveConstants.DRIVE_GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE);

        /// PID Controllers ///
        steerAnglePID = new PIDController(DriveConstants.PID_Encoder_Steer.P, DriveConstants.PID_Encoder_Steer.I, DriveConstants.PID_Encoder_Steer.D);
        steerAnglePID.enableContinuousInput(-180, 180);

        // Get the motor controller PIDs
        steerMotorVelocityPID = steerMotor.getPIDController();
        driveMotorVelocityPID = driveMotor.getPIDController();

        // set PID coefficients
        steerMotorVelocityPID.setP(DriveConstants.PID_SparkMax_Steer.P);
        steerMotorVelocityPID.setI(DriveConstants.PID_SparkMax_Steer.I);
        steerMotorVelocityPID.setD(DriveConstants.PID_SparkMax_Steer.D);
        steerMotorVelocityPID.setIZone(DriveConstants.PID_SparkMax_Steer.IZ);
        steerMotorVelocityPID.setFF(DriveConstants.PID_SparkMax_Steer.kFF);
        steerMotorVelocityPID.setOutputRange(-1, 1);
        steerMotor.burnFlash();

        // set PID coefficients
        driveMotorVelocityPID.setP(DriveConstants.PID_SparkMax_Drive.P);
        driveMotorVelocityPID.setI(DriveConstants.PID_SparkMax_Drive.I);
        driveMotorVelocityPID.setD(DriveConstants.PID_SparkMax_Drive.D);
        driveMotorVelocityPID.setIZone(DriveConstants.PID_SparkMax_Drive.IZ);
        driveMotorVelocityPID.setFF(DriveConstants.PID_SparkMax_Drive.kFF);
        driveMotorVelocityPID.setOutputRange(-1, 1);
        driveMotor.burnFlash();
    }

    /**
     * Returns the current state of the module.
     * 
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double driveSpeed = speedFromDriveRpm(driveNEOMotorEncoder.getVelocity());
        double steerAngleRadians = Math.toRadians(steerAngleEncoder.getAbsolutePosition().getValue() * 360);

        return new SwerveModuleState(driveSpeed, new Rotation2d(steerAngleRadians));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean logValues, String name) {

        double steerAngleDegrees = steerAngleEncoder.getAbsolutePosition().getValue() * 360;
        double curSteerAngleRadians = Math.toRadians(steerAngleDegrees);

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(curSteerAngleRadians));

        // The output of the steerAnglePID becomes the steer motor rpm reference.
        double steerMotorRpm = steerAnglePID.calculate(steerAngleDegrees,
                state.angle.getDegrees());

        steerMotorVelocityPID.setReference(steerMotorRpm, CANSparkMax.ControlType.kVelocity);

        double driveMotorRpm = driveRpmFromSpeed(state.speedMetersPerSecond);

        if (logValues) {
            double driveSpeed = driveNEOMotorEncoder.getVelocity();
            SmartDashboard.putNumber(name + " DriveSpeedMetersPerSecond", state.speedMetersPerSecond);
            SmartDashboard.putNumber(name + " DriveMotorRpmCommand", driveMotorRpm);
            SmartDashboard.putNumber(name + " DriveMotorSpeed", driveSpeed);
        }

        if(driveMotorVelocityPID.setReference(driveMotorRpm, CANSparkMax.ControlType.kVelocity)!=REVLibError.kOk){
            System.out.println("Bad");
        }
        SmartDashboard.putNumber(String.valueOf(driveMotorID), driveMotorRpm);
    }

    /**
     * Returns the required motor rpm from the desired wheel speed in meters/second
     * 
     * @param speedMetersPerSecond
     * @return rpm of the motor
     */
    public double driveRpmFromSpeed(double speedMetersPerSecond) {
        var rpm = speedMetersPerSecond * 60.0 / DriveConstants.WHEEL_CIRCUMFERENCE / DriveConstants.DRIVE_GEAR_RATIO;
        return -1 * rpm; // Rotation reversed due to gears.
    }

    /**
     * Returns the wheel speed in meters/second calculated from the drive motor rpm.
     * 
     * @param rpm
     * @return wheelSpeed
     */
    public double speedFromDriveRpm(double rpm) {
        var speedMetersPerSecond = rpm * DriveConstants.DRIVE_GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE / 60.0;
        return -1 * speedMetersPerSecond; // Rotation reversed due to gears.
    }

    public SwerveModulePosition getPosition() {
        double distance = driveNEOMotorEncoder.getPosition();
        return new SwerveModulePosition(distance, new Rotation2d(Math.toRadians(steerAngleEncoder.getAbsolutePosition().getValue())));
    }
}