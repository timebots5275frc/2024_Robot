package frc.robot.CustomTypes.RgbZones;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.RGB;

public abstract class RGB_Zone {
    protected int zoneStartIndex;
    protected int zoneEndIndex;
    protected final RGB rgbSubSystem;

    public RGB_Zone(int zoneStartIndex, int zoneEndIndex, RGB rgbSubSystem)
    {
        this.zoneStartIndex = zoneStartIndex;
        this.zoneEndIndex = zoneEndIndex;
        this.rgbSubSystem = rgbSubSystem;
    }

    // abstract class must have a default constructor for some reason???
    public RGB_Zone() { this.rgbSubSystem = null; }

    public void setSolidColor(Color color)
    {
        Color dimmedColor = rgbSubSystem.getDimmedColor(color);

        for (int i = zoneStartIndex; i <= zoneEndIndex; i++)
        {
            rgbSubSystem.ledBuffer.setLED(i, dimmedColor);
        }

        rgbSubSystem.setBufferDirty();
    }

    public abstract void setProgressColor(double progress, Color progressFillColor, Color backgroundColor);
}