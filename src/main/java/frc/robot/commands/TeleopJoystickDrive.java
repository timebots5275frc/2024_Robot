// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain.SwerveDrive;

public class TeleopJoystickDrive extends Command {

    public SwerveDrive drivetrain;

    private Joystick driveStick;
    private boolean fieldRelative;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param joystick  The control input for driving
     */
    public TeleopJoystickDrive(SwerveDrive _subsystem, Joystick _driveStick, boolean _fieldRelative) {
        this.drivetrain = _subsystem;
        this.driveStick = _driveStick;
        this.fieldRelative = _fieldRelative;

        addRequirements(_subsystem);
    }

    public void SetFieldRelative(boolean setboolfieldRelative) {
        System.out.println("SetFieldRelative = " + setboolfieldRelative);
        this.fieldRelative = setboolfieldRelative;
    }

    @Override
    public void initialize() {
        drivetrain.resetPigeon();
    }

    @Override
    public void execute() {

        double xSpeed = this.smartJoystick(driveStick.getY() * -1, Constants.ControllerConstants.DEADZONE_DRIVE) * Constants.DriveConstants.MAX_DRIVE_SPEED;
        double ySpeed = this.smartJoystick(driveStick.getX() * -1, Constants.ControllerConstants.DEADZONE_DRIVE) * Constants.DriveConstants.MAX_DRIVE_SPEED;
        double rotRate = this.smartJoystick(driveStick.getTwist() * -1, Constants.ControllerConstants.DEADZONE_STEER) * Constants.DriveConstants.MAX_TWIST_RATE;

        double throttle = (-driveStick.getThrottle() + 1) / 2; // between 0 and 1 = 0% and 100%

        xSpeed *= throttle;
        ySpeed *= throttle;
        rotRate *= throttle;

        SmartDashboard.putNumber("Throttle teleJoy", throttle);

        SmartDashboard.putNumber("xSpeed teleJoy smart", xSpeed);
        SmartDashboard.putNumber("ySpeed teleJoy smart ", ySpeed);
        SmartDashboard.putNumber("rotRate teleJoy smart ", rotRate);

        drivetrain.drive(xSpeed, ySpeed, rotRate, fieldRelative);

    }

    /**
     * 
     * @param _in
     * @param deadZoneSize between -1 and 1
     * @return
     */
    public double smartJoystick(double _in, double deadZoneSize) {
        if (Math.abs(_in) < deadZoneSize) {
            return 0;
        }

        if (_in > 0) {
            return (_in - deadZoneSize) / (1 - deadZoneSize);
        } else if (_in < 0) {
            return (_in + deadZoneSize) / (1 - deadZoneSize);
        }
        return 0;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void setFieldRelative(boolean bool) {
        this.fieldRelative = bool;
    }
}
