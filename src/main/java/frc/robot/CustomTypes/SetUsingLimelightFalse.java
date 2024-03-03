package frc.robot.CustomTypes;

import frc.robot.subsystems.Vision.Vision;

public class SetUsingLimelightFalse implements Runnable {
    @Override
    public void run() {
        Vision.Instance.setUsingLimelight(false);
    }
}
