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
            //double currentShooterHeight = shooter.getShooterEndHeight() + ShooterConstants.SHOOTER_OFFSET_FROM_GROUND;
            double heightDifferenceBetweenShooterAndSpeakerTargetPosition = ShooterConstants.SHOOTER_TARGET_HEIGHT - 9.5;

            Vector3 aprilTagInRobotSpace = vision.AprilTagPosInRobotSpace();
            double horizontalDistanceToAprilTag = (new Vector2(aprilTagInRobotSpace.x, aprilTagInRobotSpace.z).magnitude()) * 39.37 - 7;

            //double C = 0.0355;
            //double C = 0.034;
            //double C = 0.03475;
            double C = 0.039;

            return Math.toDegrees(Math.atan2(heightDifferenceBetweenShooterAndSpeakerTargetPosition, horizontalDistanceToAprilTag)) + Math.pow(horizontalDistanceToAprilTag, 1.08) * C;
        }
        else { return shooter.getShooterAngle(); }
    }
}
