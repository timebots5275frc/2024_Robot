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
import frc.robot.Constants.IntakeConstants;
import frc.robot.CustomTypes.RgbZones.*;
import frc.robot.subsystems.Input.Input;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Shooter.ShooterRunState;
import frc.robot.subsystems.Vision.Vision;

public class RGB extends SubsystemBase {

  static final Color RED = new Color(255, 0, 0);
  static final Color GREEN = new Color(0, 255, 0);
  static final Color BLUE = new Color(0, 0, 255);

  static final Color OFF = new Color(0, 0, 0);
  static final Color ORANGE = new Color(255, 30, 0);
  static final Color YELLOW = new Color(255, 155, 0);
  static final Color PURPLE = new Color(170, 0, 255);

  boolean bufferDirty = false;

  int periodicCalls = 0;

  double rgbStrength = .15;

  static final double startFlashTime = 30;
  static final int flashInterval = 20;
  boolean displayEndOfMatchFlash = false;

  private AddressableLED m_led = new AddressableLED(0);
  public final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(212);

  final RGB_Zone SHOOTER_RIGHT_ZONE = new RGB_Zone(0, 41, this) {

    @Override
    public void setProgressColor(double progress, Color progressFillColor, Color backgroundColor) {
      Color dimmedProgressColor = rgbSubSystem.getDimmedColor(progressFillColor);
      Color dimmedBackgroundColor = rgbSubSystem.getDimmedColor(backgroundColor);
      int oneStripLength = (zoneEndIndex - zoneStartIndex) / 2 + 1;

      for (int i = 0; i < oneStripLength; i++)
      {
        rgbSubSystem.ledBuffer.setLED(i, (double)i / oneStripLength <= progress ? dimmedProgressColor : dimmedBackgroundColor);
        rgbSubSystem.ledBuffer.setLED(i + oneStripLength, 1 - ((double)i + 1) / oneStripLength <= progress ? dimmedProgressColor : dimmedBackgroundColor);
      }

      rgbSubSystem.setBufferDirty();
    }
  };

  final RGB_Zone SHOOTER_LEFT_ZONE = new RGB_Zone(42, 83, this) {

    @Override
    public void setProgressColor(double progress, Color progressFillColor, Color backgroundColor) {
      Color dimmedProgressColor = rgbSubSystem.getDimmedColor(progressFillColor);
      Color dimmedBackgroundColor = rgbSubSystem.getDimmedColor(backgroundColor);
      int oneStripLength = (zoneEndIndex - zoneStartIndex) / 2 + 1;

      for (int i = 0; i < oneStripLength; i++)
      {
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i, (double)i / oneStripLength <= progress ? dimmedProgressColor : dimmedBackgroundColor);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i + oneStripLength, 1 - ((double)i + 1)  / oneStripLength <= progress ? dimmedProgressColor : dimmedBackgroundColor);
      }

      rgbSubSystem.setBufferDirty();
    }
  };

  final RGB_Zone CLIMBER_RIGHT_ZONE = new RGB_Zone(84, 147, this) {

    @Override
    public void setProgressColor(double progress, Color progressFillColor, Color backgroundColor) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setProgressColor'");
    }
  };

  final RGB_Zone CLIMBER_LEFT_ZONE = new RGB_Zone(148, 211, this) {

    @Override
    public void setProgressColor(double progress, Color progressFillColor, Color backgroundColor) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setProgressColor'");
    }
  };

      Shooter shooter;
      Intake intake;
      private double shooterSpeed;

      public RGB(Shooter shooter, Intake intake) {
      // PWM port 9
      // Must be a PWM header, not MXP or DIO
      this.shooter = shooter;
      this.intake = intake;

      m_led.setLength(ledBuffer.getLength());

      setColorPattern(new Color[] { BLUE, BLUE, RED, RED } );
      SHOOTER_RIGHT_ZONE.setSolidColor(ORANGE);
      SHOOTER_LEFT_ZONE.setSolidColor(RED);
      CLIMBER_LEFT_ZONE.setSolidColor(PURPLE);
      CLIMBER_RIGHT_ZONE.setSolidColor(YELLOW);
      m_led.start();
    }

    public Color getDimmedColor(Color color)
    {
      return new Color(color.red * rgbStrength, color.green * rgbStrength, color.blue * rgbStrength);
    }

    public Color multiplyColor(Color color, double a) { return new Color(color.red * a, color.green * a, color.blue * a); }

    public void setSolidRGBColor(Color color)
    {
      Color dimmedColor = multiplyColor(color, rgbStrength);

      for (int i = 0; i < ledBuffer.getLength(); i ++) {
        ledBuffer.setLED(i, dimmedColor);
      }

      m_led.setData(ledBuffer);
    }

    public void setColorPattern(Color[] colors)
    {
      Color[] dimmedColors = new Color[colors.length];

      for (int i = 0; i < dimmedColors.length; i++)
      {
        dimmedColors[i] = multiplyColor(colors[i], rgbStrength);
      }

      for (int i = 0; i < ledBuffer.getLength(); i ++) {
        Color color = dimmedColors[i % dimmedColors.length];
        ledBuffer.setLED(i, color);
      }

      m_led.setData(ledBuffer);
    }


  public void setAllianceColor() {
      if (DriverStation.getAlliance().get() == Alliance.Red) { setSolidRGBColor(RED); } 
      else if (DriverStation.getAlliance().get() == Alliance.Blue) { setSolidRGBColor(BLUE); }
      else { setColorPattern(new Color[] { RED, BLUE }); }
  }

  public void rainbowRGB()
  {
      for (int i = 0; i < ledBuffer.getLength(); i ++) {
        ledBuffer.setHSV(i, periodicCalls + i % 180, 255, (int)(255 * rgbStrength));
      }

      m_led.setData(ledBuffer);
  }

  public void allRandom()
  {
      for (int i = 0; i < ledBuffer.getLength(); i ++) {
        ledBuffer.setRGB(i, (int)(Math.random() * 255 * rgbStrength), (int)(Math.random() * 255 * rgbStrength), (int)(Math.random() * 255 * rgbStrength));
      }

      m_led.setData(ledBuffer);
  }

  void SetAscendingOrderRGB()
  {
      for (int i = 0; i < ledBuffer.getLength(); i ++) {
        int val = i;
        ledBuffer.setRGB(i, val, val, val);
      }

      m_led.setData(ledBuffer);
  }

  public void setBufferDirty() { bufferDirty = true; }

  @Override
  public void periodic() {
    // //rainbowRGB();
    // periodicCalls++;

    // double matchTime = DriverStation.getMatchTime();
    // if (matchTime < startFlashTime && DriverStation.isTeleop() && periodicCalls % flashInterval == 0) { displayEndOfMatchFlash = !displayEndOfMatchFlash; }
    // if (matchTime > startFlashTime || matchTime == -1 || !DriverStation.isTeleop()) { displayEndOfMatchFlash = false; }

    // if (displayEndOfMatchFlash) {  setSolidRGBColor(YELLOW); }
    // //else if (shooter.getCurrentRunState() == ShooterRunState.SHOOT) { shooterLED(); }
    // //else if (Vision.usingLimelight) { setSolidRGBColor(GREEN); }
    // else if (intake.limitSwitchPressed()) { setSolidRGBColor(ORANGE); }
    // else if (intake.getCurrentPivotState() == IntakePivotState.OUT || intake.getCurrentPivotAngle() < IntakeConstants.INTAKE_UP_POS) { setSolidRGBColor(PURPLE); }
    // else { setAllianceColor(); }

    SHOOTER_RIGHT_ZONE.setProgressColor(Input.Throttle, GREEN, BLUE);
    SHOOTER_LEFT_ZONE.setProgressColor(Input.Throttle, GREEN, BLUE);

    if (bufferDirty) {
      m_led.setData(ledBuffer);
      bufferDirty = false;
    }
  }
}