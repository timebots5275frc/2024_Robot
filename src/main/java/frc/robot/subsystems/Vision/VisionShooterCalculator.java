package frc.robot.subsystems.Vision;

import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.CustomTypes.Math.Vector3;

public class VisionShooterCalculator {
    static Vision vision;
    static Shooter shooter;

    public static void SetVisionReference(Vision vision) { VisionShooterCalculator.vision = vision; }
    public static void SetShooterReference(Shooter shooter) { VisionShooterCalculator.shooter = shooter; }

    public static double GetSpeakerShooterAngle()
    {
        if (vision.hasValidData())
        {
            double currentShooterHeight = 0;
            double heightDifferenceBetweenShooterAndSpeakerTargetPosition = ShooterConstants.SHOOTER_TARGET_HEIGHT - currentShooterHeight;

            Vector3 aprilTagInRobotSpace = vision.AprilTagPosInRobotSpace();
            double horizontalDistanceToAprilTag = new Vector2(aprilTagInRobotSpace.x, aprilTagInRobotSpace.z).magnitude();

            double C = 0.001;

            return Math.toDegrees(Math.tan(heightDifferenceBetweenShooterAndSpeakerTargetPosition / horizontalDistanceToAprilTag)) + horizontalDistanceToAprilTag * C;
        }
        else { return shooter.getShooterAngle(); }
    }
}
