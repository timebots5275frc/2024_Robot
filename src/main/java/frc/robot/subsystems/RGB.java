// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RGB extends SubsystemBase {
  private AddressableLED m_led = new AddressableLED(0);
      // Reuse buffer
      // Default to a length of 60, start empty output
      // Length is expensive to set, so only set it once, then just update data
      private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(85);

      Shooter shooter;
      private double shooterSpeed;

      public RGB(Shooter shooter) {
      // PWM port 9
      // Must be a PWM header, not MXP or DIO
      this.shooter = shooter;
      m_led.setLength(m_ledBuffer.getLength());
  
      // Set the data
      for (int i = 0; i < m_ledBuffer.getLength(); i += 2) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 255, 0, 0);
      }
      for (int i = 1; i < m_ledBuffer.getLength(); i += 2) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 0, 0, 255);
      }
      

      m_led.start();
      m_led.setData(m_ledBuffer);

    }

  public void setBlueLed()
  {
    for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 0, 0, 255);
     }
      m_led.setData(m_ledBuffer);
  }
  public void setgreen()
  {
   for(int i = 0; i < m_ledBuffer.getLength(); i ++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 0, 255, 0);
     }
      m_led.setData(m_ledBuffer);
  }
  public void setYellowLed()
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i ++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 204, 159, 47);
        System.out.println("Set led" + i);
     }
      m_led.setData(m_ledBuffer);
  }
  public void setBlinkingYellow()
  {
    for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 204, 159, 47);
        m_led.setData(m_ledBuffer);
     }
     for (int i = 0; i < m_ledBuffer.getLength(); i += 2) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 0, 0, 0);
        m_led.setData(m_ledBuffer);
     }
   
  }
  public void shooterLED()
      {
        if(shooterSpeed>0)
        {
          for(int i = 0; i < 3; i++)
          {
            m_ledBuffer.setRGB(i, 0, 0, 255);
            m_ledBuffer.setRGB(43-i, 0, 0, 255);
            m_ledBuffer.setRGB(i + 42, 0, 0, 255);
            m_ledBuffer.setRGB(84- i, 0, 0, 255);
          }
        }
        if(shooterSpeed>700)
        {
          for(int i = 4; i < 9; i++)
          {
            m_ledBuffer.setRGB(i, 0, 255, 0);
            m_ledBuffer.setRGB(43-i, 0, 255, 0);
            m_ledBuffer.setRGB(i + 42, 0, 255, 0);
            m_ledBuffer.setRGB(85- i, 0, 255, 0);
          }
        }
        if(shooterSpeed>2000)
        {
          for(int i = 9; i < 14; i++)
          {
            m_ledBuffer.setRGB(i, 0, 255, 0);
            m_ledBuffer.setRGB(43-i, 0, 255, 0);
            m_ledBuffer.setRGB(i + 42, 0, 255, 0);
            m_ledBuffer.setRGB(85- i, 0, 255, 0);
          }
        }
        if(shooterSpeed>3200)
        {
          for(int i = 14; i < 17; i++)
          {
            m_ledBuffer.setRGB(i, 0, 255, 0);
            m_ledBuffer.setRGB(43-i, 0, 255, 0);
            m_ledBuffer.setRGB(i + 42, 0, 255, 0);
            m_ledBuffer.setRGB(85- i, 0, 255, 0);
          }
        }
        if(shooterSpeed>4600)
          {
            for(int i = 17; i < 22; i++)
            {
             m_ledBuffer.setRGB(i, 255, 0, 0);
             m_ledBuffer.setRGB(43-i, 255, 0, 0);
             m_ledBuffer.setRGB(i + 42, 255, 0, 0);
             m_ledBuffer.setRGB(85- i, 255, 0, 0);
           }
           m_led.setData(m_ledBuffer);
          }
        }
      

  public void startLight() {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
         for (int i = 0; i < m_ledBuffer.getLength(); i++) {
         // Sets the specified LED to the RGB values for red
         m_ledBuffer.setRGB(i, 255, 0, 0);
         }
      } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
         for (int i = 0; i < m_ledBuffer.getLength(); i++) {
         // Sets the specified LED to the RGB values for red
         m_ledBuffer.setRGB(i, 0, 0, 255);
         }
      }
      m_led.setData(m_ledBuffer);
  }

  @Override
  
  public void periodic() {
    shooterSpeed = shooter.getShooterRPM();
    shooterLED();
    if (!DriverStation.isEnabled()) {
      startLight();
    }
  }
}