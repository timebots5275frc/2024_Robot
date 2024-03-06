package frc.robot.CustomTypes;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.RGB;

public class RGB_Zone {
    protected int zoneStartIndex;
    protected int zoneEndIndex;
    protected final RGB rgbSubSystem;

    public RGB_Zone(int zoneStartIndex, int zoneEndIndex, RGB rgbSubSystem)
    {
        this.zoneStartIndex = zoneStartIndex;
        this.zoneEndIndex = zoneEndIndex;
        this.rgbSubSystem = rgbSubSystem;
    }

    public void setSolidColor(Color color)
    {
        for (int i = zoneStartIndex; i <= zoneEndIndex; i++)
        {
            rgbSubSystem.ledBuffer.setLED(i, color);
        }

        rgbSubSystem.setBufferDirty();
    }
}
