// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RGB extends SubsystemBase {

  static final Color RED = new Color(255, 0, 0);
  static final Color GREEN = new Color(0, 255, 0);
  static final Color BLUE = new Color(0, 0, 255);

  static final Color OFF = new Color(0, 0, 0);
  static final Color YELLOW = new Color(204, 159, 47);

  private AddressableLED m_led = new AddressableLED(0);
      // Reuse buffer
      // Default to a length of 60, start empty output
      // Length is expensive to set, so only set it once, then just update data
      private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(151);

      Shooter shooter;
      private double shooterSpeed;

      public RGB(Shooter shooter) {
      // PWM port 9
      // Must be a PWM header, not MXP or DIO
      this.shooter = shooter;
      m_led.setLength(m_ledBuffer.getLength());
  
      setColorPattern(new Color[] { BLUE, RED } );

      m_led.start();
    }

    public void setSolidRGBColor(Color color)
    {
      for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
        m_ledBuffer.setRGB(i, (int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
      }

      m_led.setData(m_ledBuffer);
    }

    public void setColorPattern(Color[] colors)
    {
      for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
        Color color = colors[i % colors.length];
        m_ledBuffer.setRGB(i, (int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
      }

      m_led.setData(m_ledBuffer);
    }

  public void shooterLED()
      {
        if(shooterSpeed<4000)
        {
          for(int i = 1; i < 22; i++)
          {
            m_ledBuffer.setRGB(i, 0, 0, 255);
            m_ledBuffer.setRGB(43-i, 0, 0, 255);
            m_ledBuffer.setRGB(i + 42, 0, 0, 255);
            m_ledBuffer.setRGB(85- i, 0, 0, 255);
          }
          m_led.setData(m_ledBuffer);
        }
        else if(shooterSpeed<3000)
        {
          setAllianceColor()
          for(int i = 1; i < 17; i++)
          {
            m_ledBuffer.setRGB(i, 0, 255, 0);
            m_ledBuffer.setRGB(43-i, 0, 255, 0);
            m_ledBuffer.setRGB(i + 42, 0, 255, 0);
            m_ledBuffer.setRGB(85- i, 0, 255, 0);
          }
          m_led.setData(m_ledBuffer);
        }
        else if(shooterSpeed<2000)
        {
          setAllianceColor()
          for(int i = 1; i < 12; i++)
          {
            m_ledBuffer.setRGB(i, 0, 255, 0);
            m_ledBuffer.setRGB(43-i, 0, 255, 0);
            m_ledBuffer.setRGB(i + 42, 0, 255, 0);
            m_ledBuffer.setRGB(85- i, 0, 255, 0);
          }
          m_led.setData(m_ledBuffer);
        }
        else if(shooterSpeed<1000)
        {
          setAllianceColor()
          for(int i = 1; i < 9; i++)
          {
            m_ledBuffer.setRGB(i, 0, 255, 0);
            m_ledBuffer.setRGB(43-i, 0, 255, 0);
            m_ledBuffer.setRGB(i + 42, 0, 255, 0);
            m_ledBuffer.setRGB(85- i, 0, 255, 0);
          }
          m_led.setData(m_ledBuffer);
        }
        else()
          {
            setAllianceColor()
            for(int i = 1; i < 4; i++)
            {
             m_ledBuffer.setRGB(i, 255, 0, 0);
             m_ledBuffer.setRGB(43-i, 255, 0, 0);
             m_ledBuffer.setRGB(i + 42, 255, 0, 0);
             m_ledBuffer.setRGB(85- i, 255, 0, 0);
           }
           m_led.setData(m_ledBuffer);
          }
        }
      

  public void setAllianceColor() {
      if (DriverStation.getAlliance().get() == Alliance.Red) { setSolidRGBColor(RED); } 
      else if (DriverStation.getAlliance().get() == Alliance.Blue) { setSolidRGBColor(BLUE); }
      else { setColorPattern(new Color[] { RED, BLUE }); }
  }

  public void rainbowRGB()
  {
      for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
        Color color = new Color(1, 1, 1);
        m_ledBuffer.setRGB(i, (int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
      }

      m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {

  }
}