// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.Random;
import java.util.random.RandomGenerator;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Shooter.ShooterRunState;
import frc.robot.subsystems.Vision.Vision;

public class RGB extends SubsystemBase {

  static final Color RED = new Color(255, 0, 0);
  static final Color GREEN = new Color(0, 255, 0);
  static final Color BLUE = new Color(0, 0, 255);

  static final Color OFF = new Color(0, 0, 0);
  static final Color ORANGE = new Color(255, 30, 0);
  static final Color YELLOW = new Color(204, 159, 47);
  static final Color PURPLE = new Color(170, 0, 255);

  int periodicCalls = 0;

  double rgbStrength = .5;

  static final double startFlashTime = 15;
  static final int flashInterval = 20;
  boolean displayEndOfMatchFlash = false;

  private AddressableLED m_led = new AddressableLED(0);
      // Reuse buffer
      // Default to a length of 60, start empty output
      // Length is expensive to set, so only set it once, then just update data
      private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(213);

      Shooter shooter;
      Intake intake;
      private double shooterSpeed;

      public RGB(Shooter shooter, Intake intake) {
      // PWM port 9
      // Must be a PWM header, not MXP or DIO
      this.shooter = shooter;
      this.intake = intake;

      m_led.setLength(m_ledBuffer.getLength());
      setColorPattern(new Color[] { BLUE, BLUE, RED, RED } );
      m_led.start();
    }

    public Color multiplyColor(Color color, double a) { return new Color(color.red * a, color.green * a, color.blue * a); }

    public void setSolidRGBColor(Color color)
    {
      Color dimmedColor = multiplyColor(color, rgbStrength);

      for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
        m_ledBuffer.setLED(i, dimmedColor);
      }

      m_led.setData(m_ledBuffer);
    }

    public void setColorPattern(Color[] colors)
    {
      Color[] dimmedColors = new Color[colors.length];

      for (int i = 0; i < dimmedColors.length; i++)
      {
        dimmedColors[i] = multiplyColor(colors[i], rgbStrength);
      }

      for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
        Color color = dimmedColors[i % dimmedColors.length];
        m_ledBuffer.setLED(i, color);
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
          setAllianceColor();
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
          setAllianceColor();
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
          setAllianceColor();
          for(int i = 1; i < 9; i++)
          {
            m_ledBuffer.setRGB(i, 0, 255, 0);
            m_ledBuffer.setRGB(43-i, 0, 255, 0);
            m_ledBuffer.setRGB(i + 42, 0, 255, 0);
            m_ledBuffer.setRGB(85- i, 0, 255, 0);
          }
          m_led.setData(m_ledBuffer);
        }
        else
          {
            setAllianceColor();
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
        m_ledBuffer.setHSV(i, periodicCalls + i % 180, 255, (int)(255 * rgbStrength));
      }

      m_led.setData(m_ledBuffer);
  }

  public void allRandom()
  {
      for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
        m_ledBuffer.setRGB(i, (int)(Math.random() * 255 * rgbStrength), (int)(Math.random() * 255 * rgbStrength), (int)(Math.random() * 255 * rgbStrength));
      }

      m_led.setData(m_ledBuffer);
  }

  void SetAscendingOrderRGB()
  {
      for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
        int val = i;
        m_ledBuffer.setRGB(i, val, val, val);
      }

      m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    //rainbowRGB();
    periodicCalls++;

    double matchTime = DriverStation.getMatchTime();
    if (matchTime < startFlashTime && DriverStation.isTeleop() && periodicCalls % flashInterval == 0) { displayEndOfMatchFlash = !displayEndOfMatchFlash; }
    if (matchTime > startFlashTime || matchTime == -1 || !DriverStation.isTeleop()) { displayEndOfMatchFlash = false; }

    if (displayEndOfMatchFlash) {  setSolidRGBColor(YELLOW); }
    //else if (shooter.getCurrentRunState() == ShooterRunState.SHOOT) { shooterLED(); }
    else if (Vision.usingVisionCommand) { setSolidRGBColor(GREEN); }
    else if (intake.limitSwitchPressed()) { setSolidRGBColor(ORANGE); }
    else if (intake.getCurrentPivotState() == IntakePivotState.OUT) { setSolidRGBColor(PURPLE); }
    else { setAllianceColor(); }
  }
}