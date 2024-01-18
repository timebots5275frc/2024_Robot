package frc.robot.CustomTypes;

public class PID_Values {
    public double P;
    public double I;
    public double D;
    public double IZ;
    public double KFF;

    public PID_Values(double P_, double I_, double D_)
    {
        P = P_;
        I = I_;
        D = D_;
        IZ = 0.0;
        KFF = 0.0;
    }

    public PID_Values(double P_, double I_, double D_, double IZ_, double KFF_)
    {
        P = P_;
        I = I_;
        D = D_;
        IZ = IZ_;
        KFF = KFF_;
    }
}
