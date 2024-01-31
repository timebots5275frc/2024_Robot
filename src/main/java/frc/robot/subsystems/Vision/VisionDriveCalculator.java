package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CustomTypes.Math.SillyMath;
import frc.robot.CustomTypes.Math.Vector2;

public class VisionDriveCalculator {
    static Vision vision;

    public static void SetVisionReference(Vision vision) { VisionDriveCalculator.vision = vision; }

    public static double rotateTowardsTarget()
    {
        return -SillyMath.clamp(vision.HorizontalOffsetFromAprilTag() / 22, -1, 1);
    }

    public static AprilTagMoveVelocity GetVelocityToAprilTagOffset(Vector2 aprilTagOffset)
    {
        if (vision.hasValidData())
        {
            Vector2 adjustedAprilTagOffset = new Vector2(-aprilTagOffset.x, aprilTagOffset.y); // needed because otherwise doesnt work right for some reason
            Vector2 aprilTagOffsetInRobotSpace = Vector2.rotate(adjustedAprilTagOffset, Math.toRadians(vision.AprilTagRotInRobotSpace().y + 180));
            Vector2 aprilTagInRobotSpace = new Vector2(vision.AprilTagPosInRobotSpace().x, vision.AprilTagPosInRobotSpace().z);
            Vector2 targetPositionInRobotSpace = aprilTagInRobotSpace.add(aprilTagOffsetInRobotSpace);

            SmartDashboard.putString("AT Offset in Robot Space", aprilTagOffsetInRobotSpace.toString(3));
            SmartDashboard.putString("AT in robot space", aprilTagInRobotSpace.toString(3));
            SmartDashboard.putString("Target Position in Robot Space", targetPositionInRobotSpace.toString(3));
            SmartDashboard.putNumber("Magnientuecd", targetPositionInRobotSpace.magnitude());

            return new AprilTagMoveVelocity(ApplyEasingToVector2(targetPositionInRobotSpace), targetPositionInRobotSpace.magnitude(), true);
        }

        return new AprilTagMoveVelocity(Vector2.zero, 0, false);
    }

    static Vector2 ApplyEasingToVector2(Vector2 direction)
    {
        double easedMagnitude = Math.cbrt(direction.magnitude() + 1) - 1;

        return direction.normalized().times(easedMagnitude);
    }

    public static class AprilTagMoveVelocity
    {
        public Vector2 velocity;
        public double distanceFromTarget;
        public boolean validData;

        public AprilTagMoveVelocity(Vector2 velocity, double distanceFromTarget, boolean validData)
        {
            this.velocity = velocity;
            this.distanceFromTarget = distanceFromTarget;
            this.validData = validData;
        }
    }
}
