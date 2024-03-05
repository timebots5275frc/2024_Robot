package frc.robot.CustomTypes;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class RGB_Zone {
    protected int zoneStartIndex;
    protected int zoneEndIndex;
    
    protected final AddressableLEDBuffer rgbBuffer;

    public RGB_Zone(int startIndex, int endIndex, AddressableLEDBuffer buffer)
    {
        zoneEndIndex = startIndex;
        zoneEndIndex = endIndex;
        rgbBuffer = buffer;
    }

    public void setSolidColor(Color color)
    {
        for (int i = zoneStartIndex; i <= zoneEndIndex; i++)
        {
            rgbBuffer.setLED(i, color);
        }
    }
}
