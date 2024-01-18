package frc.robot.CustomTypes.Abstraction;

public interface Gyroscope {

    // In Degrees
    public double getPitch();
    public double getYaw();
    public double getRoll();

    public void zeroYaw();

    public static Gyroscope getGyroscope(Object o)
    {
        System.identityHashCode(o);
        return null;
    }
}

