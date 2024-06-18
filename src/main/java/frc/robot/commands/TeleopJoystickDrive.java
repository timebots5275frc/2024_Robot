// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.AutoTargetStateManager;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionDriveCalculator;
import frc.robot.subsystems.Vision.VisionFollowAprilTag;
import frc.robot.subsystems.Input.Input;
import frc.robot.CustomTypes.Math.SillyMath;
import frc.robot.CustomTypes.Math.Vector2;

public class TeleopJoystickDrive extends Command {

    private SwerveDrive drivetrain;
    private Input input;
    private VisionFollowAprilTag vision_followAprilTag;

    private Joystick driveStick;
    private boolean fieldRelative;
    private double C;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param joystick  The control input for driving
     */
    public TeleopJoystickDrive(SwerveDrive _swerveDrive, Joystick _driveStick, Input input_, boolean _fieldRelative) {
        this.drivetrain = _swerveDrive;
        this.driveStick = _driveStick;
        this.input = input_;
        this.fieldRelative = _fieldRelative;
        this.vision_followAprilTag = new VisionFollowAprilTag(_swerveDrive);

        addRequirements(_swerveDrive);

        C = 6;
    }

    public void SetFieldRelative(boolean setboolfieldRelative) {
        System.out.println("SetFieldRelative = " + setboolfieldRelative);
        this.fieldRelative = setboolfieldRelative;
    }

    @Override
    public void initialize() {
        AutoTargetStateManager.onStart();
        //drivetrain.resetPigeon();
    }

    @Override
    public void execute() {
        Vector2 joystickInput = input.JoystickInput();
        double joystickTwist = input.JoystickTwist();
        double throttle = (-driveStick.getThrottle() + 1) / 2; // between 0 and 1 = 0% and 100%

        Vector2 inputVelocity = joystickInput.times(throttle * Constants.DriveConstants.MAX_DRIVE_SPEED);
        double inputRotationVelocity = joystickTwist * throttle * Constants.DriveConstants.MAX_TWIST_RATE;
        
        SmartDashboard.putNumber("Throttle teleJoy", throttle);
        SmartDashboard.putNumber("y velo", inputVelocity.y);


        if (!AutoTargetStateManager.isAutoTargeting) {
            drivetrain.drive(inputVelocity.x, inputVelocity.y, inputRotationVelocity, fieldRelative);
        } else {
            double turnVelocity;
            if (Constants.VisionConstants.AprilTagData.isSpeakerTag(Constants.VisionConstants.AprilTagData.getTag(Vision.Instance.AprilTagID()))) {
                double turnDirection = VisionDriveCalculator.rotateTowardsTarget(VisionDriveCalculator.getAngleOffsetForVision());
                int sign;
                if(turnDirection==0){
                    sign=0;
                } else if(turnDirection>0){
                    sign=1;
                } else{
                    sign=-1;
                }
                double dis_to_tag = Math.sqrt(Math.pow(Vision.Instance.AprilTagPosInRobotSpace().x,2)+Math.pow(Vision.Instance.AprilTagPosInRobotSpace().z,2));
                double x = Math.pow(1.3*dis_to_tag,1.2);
                double k = SillyMath.clamp(Math.abs(inputVelocity.y*.5),1,4); 
                turnVelocity = ((turnDirection*C)/x)*k;
                turnVelocity = SillyMath.clamp(turnVelocity,-4,4);
                SmartDashboard.putNumber("dis_to_tag", dis_to_tag);
                SmartDashboard.putNumber("turn_velo", turnVelocity);
                if (Math.abs(turnVelocity) < .05) { turnVelocity = 0; }
            } else {
                turnVelocity = .001;
            }
            drivetrain.drive(inputVelocity.x, inputVelocity.y, turnVelocity, fieldRelative);
        }
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
