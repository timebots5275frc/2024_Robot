// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
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
import frc.robot.Constants.ShooterConstants;
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
  static final Color NEON_PINK = new Color(255, 16, 240);

  boolean bufferDirty = false;

  int periodicCalls = 0;

  double rgbStrength = .15;

  static final double endOfMatchStartFlashTime = 30;
  boolean startedEndOfMatchFlash = false;
  boolean limitSwitchLastPeriodic = false;

  private AddressableLED m_led = new AddressableLED(0);
  public final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(212);

  ArrayList<RgbFlashCommand> flashCommands = new ArrayList<>(3);
  Color currentFlashColor = OFF;

  //#region rgb zones

  final RGB_Zone SHOOTER_RIGHT_ZONE = new RGB_Zone(0, 41, this) {
    int oneStripLength = 21;

    @Override
    public void setProgressColor(double progress, Color progressFillColor, Color backgroundColor) {
      Color dimmedProgressColor = rgbSubSystem.getDimmedColor(progressFillColor);
      Color dimmedBackgroundColor = rgbSubSystem.getDimmedColor(backgroundColor);

      for (int i = 0; i < oneStripLength; i++)
      {
        float val = (float)i / (oneStripLength - 1);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i, val <= progress ? dimmedProgressColor : dimmedBackgroundColor);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i + oneStripLength, 1 - val <= progress ? dimmedProgressColor : dimmedBackgroundColor);
      }

      rgbSubSystem.setBufferDirty();
    }
  };

  final RGB_Zone SHOOTER_LEFT_ZONE = new RGB_Zone(42, 83, this) {
    int oneStripLength = 21;

    @Override
    public void setProgressColor(double progress, Color progressFillColor, Color backgroundColor) {
      Color dimmedProgressColor = rgbSubSystem.getDimmedColor(progressFillColor);
      Color dimmedBackgroundColor = rgbSubSystem.getDimmedColor(backgroundColor);

      for (int i = 0; i < oneStripLength; i++)
      {
        float val = (float)i / (oneStripLength - 1);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i, val <= progress ? dimmedProgressColor : dimmedBackgroundColor);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i + oneStripLength, 1 - val <= progress ? dimmedProgressColor : dimmedBackgroundColor);
      }

      rgbSubSystem.setBufferDirty();
    }
  };

  final RGB_Zone CLIMBER_RIGHT_ZONE = new RGB_Zone(84, 147, this) {
    int oneStripLength = 16;

    @Override
    public void setProgressColor(double progress, Color progressFillColor, Color backgroundColor) {
      Color dimmedProgressColor = rgbSubSystem.getDimmedColor(progressFillColor);
      Color dimmedBackgroundColor = rgbSubSystem.getDimmedColor(backgroundColor);

      for (int i = 0; i < oneStripLength; i++)
      {
        float val = (float)i / (oneStripLength - 1);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i, val <= progress ? dimmedProgressColor : dimmedBackgroundColor);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i + oneStripLength, 1 - val <= progress ? dimmedProgressColor : dimmedBackgroundColor);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i + oneStripLength * 2, val <= progress ? dimmedProgressColor : dimmedBackgroundColor);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i + oneStripLength * 3, 1 - val <= progress ? dimmedProgressColor : dimmedBackgroundColor);
      }
    }
  };

  final RGB_Zone CLIMBER_LEFT_ZONE = new RGB_Zone(148, 211, this) {
    int oneStripLength = 16;

    @Override
    public void setProgressColor(double progress, Color progressFillColor, Color backgroundColor) {
      Color dimmedProgressColor = rgbSubSystem.getDimmedColor(progressFillColor);
      Color dimmedBackgroundColor = rgbSubSystem.getDimmedColor(backgroundColor);

      for (int i = 0; i < oneStripLength; i++)
      {
        float val = (float)i / (oneStripLength - 1);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i, val <= progress ? dimmedProgressColor : dimmedBackgroundColor);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i + oneStripLength, 1 - val <= progress ? dimmedProgressColor : dimmedBackgroundColor);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i + oneStripLength * 2, val <= progress ? dimmedProgressColor : dimmedBackgroundColor);
        rgbSubSystem.ledBuffer.setLED(zoneStartIndex + i + oneStripLength * 3, 1 - val <= progress ? dimmedProgressColor : dimmedBackgroundColor);
      }
    }
  };

  //#endregion

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


  public Color getAllianceColor() {
      if (DriverStation.getAlliance().get() == Alliance.Red) { return RED; } 
      else if (DriverStation.getAlliance().get() == Alliance.Blue) { return BLUE; }
      else { return OFF; }
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
    periodicCalls++;
    updateFlashCommands();

    if (shouldUseFlash()) {
      setSolidRGBColor(currentFlashColor);
      return;
    }

    Color backgroundColor = OFF;

    if (DriverStation.isTeleop())
    {
      double matchTime = DriverStation.getMatchTime();

      if (!startedEndOfMatchFlash && matchTime < endOfMatchStartFlashTime)
      {
        flashCommands.add(new RgbFlashCommand(YELLOW, 30, 20));
      }
    }

    if (!limitSwitchLastPeriodic && intake.limitSwitchPressed()) {flashCommands.add(new RgbFlashCommand(ORANGE, 1, 12)); }

    //else if (shooter.getCurrentRunState() == ShooterRunState.SHOOT) { shooterLED(); }
    //else if (Vision.usingLimelight) { setSolidRGBColor(GREEN); }
    if (intake.limitSwitchPressed()) { backgroundColor = ORANGE; }
    else if (intake.getCurrentPivotState() == IntakePivotState.OUT || intake.getCurrentPivotAngle() < IntakeConstants.INTAKE_UP_POS) { backgroundColor = PURPLE; }
    else { backgroundColor = getAllianceColor(); }

    setSolidRGBColor(backgroundColor);

    double shooterRPMPercentOfMax = shooter.getShooterRPM() / ShooterConstants.LEFT_SHOOTER_SPEED;
    if (shooterRPMPercentOfMax > 0) {
      SHOOTER_RIGHT_ZONE.setProgressColor(shooterRPMPercentOfMax, NEON_PINK, backgroundColor);
      SHOOTER_LEFT_ZONE.setProgressColor(shooterRPMPercentOfMax, NEON_PINK, backgroundColor);
    }

    limitSwitchLastPeriodic = intake.limitSwitchPressed();
    if (bufferDirty) {
      m_led.setData(ledBuffer);
      bufferDirty = false;
    }
  }

  public static Color hsvToRgb(float hue, float saturation, float value) {
    int[] rgb = new int[3];

    int hi = (int) (Math.floor(hue / 60) % 6);
    float f = (hue / 60) - (float) Math.floor(hue / 60);
    float p = value * (1 - saturation);
    float q = value * (1 - f * saturation);
    float t = value * (1 - (1 - f) * saturation);

    switch (hi) {
        case 0:
            rgb[0] = (int) (value * 255);
            rgb[1] = (int) (t * 255);
            rgb[2] = (int) (p * 255);
            break;
        case 1:
            rgb[0] = (int) (q * 255);
            rgb[1] = (int) (value * 255);
            rgb[2] = (int) (p * 255);
            break;
        case 2:
            rgb[0] = (int) (p * 255);
            rgb[1] = (int) (value * 255);
            rgb[2] = (int) (t * 255);
            break;
        case 3:
            rgb[0] = (int) (p * 255);
            rgb[1] = (int) (q * 255);
            rgb[2] = (int) (value * 255);
            break;
        case 4:
            rgb[0] = (int) (t * 255);
            rgb[1] = (int) (p * 255);
            rgb[2] = (int) (value * 255);
            break;
        case 5:
            rgb[0] = (int) (value * 255);
            rgb[1] = (int) (p * 255);
            rgb[2] = (int) (q * 255);
            break;
        default:
            break;
    }

    return new Color(rgb[0], rgb[1], rgb[2]);
  }

  void updateFlashCommands()
  {
    for (int i = flashCommands.size() - 1; i > -1; i--)
    {
      flashCommands.get(i).periodicUpdate();

      if (flashCommands.get(i).shouldDelete()) { flashCommands.remove(i); }
    }
  }

  boolean shouldUseFlash()
  {
    for (int i = 0; i < flashCommands.size(); i++)
    {
      if (flashCommands.get(i).displayFlash())
      {
        currentFlashColor = flashCommands.get(i).flashColor();
        return true;
      }
    }

    return false;
  }

  class RgbFlashCommand {
    final Color flashColor;
    final double flashTime;
    final int flashInterval;

    double currentTime;
    int periodicCalls;
    boolean displayFlash;

    public RgbFlashCommand(Color flashColor, double flashTime, int flashInterval)
    {
      this.flashColor = flashColor;
      this.flashTime = flashTime;
      this.flashInterval = flashInterval;

      currentTime = 0;
      periodicCalls = 0;
      displayFlash = true;
    }

    public void periodicUpdate()
    {
      periodicCalls++;
      currentTime += .02;

      if (periodicCalls % flashInterval == 0) { displayFlash = !displayFlash; }
    }

    public boolean shouldDelete() { return currentTime > flashTime; }
    public Color flashColor() { return flashColor; }
    public boolean displayFlash() { return displayFlash; }
  }
}