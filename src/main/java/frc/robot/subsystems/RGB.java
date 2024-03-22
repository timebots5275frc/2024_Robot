// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.CustomTypes.RgbZones.RGB_Zone;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Shooter.ShooterRunState;
import frc.robot.subsystems.Vision.Vision;

public class RGB extends SubsystemBase {

  static final Color RED = new Color(255, 0, 0);
  static final Color GREEN = new Color(0, 255, 0);
  static final Color BLUE = new Color(0, 0, 255);

  static final Color DARK_RED = new Color(150, 0, 0);
  static final Color DARK_GREEN = new Color(0, 150, 0);
  static final Color DARK_BLUE = new Color(0, 0, 150);

  static final Color OFF = new Color(0, 0, 0);
  static final Color ORANGE = new Color(255, 30, 0);
  static final Color YELLOW = new Color(255, 155, 0);
  static final Color PURPLE = new Color(170, 0, 255);
  static final Color NEON_PINK = new Color(255, 16, 240);

  static Color CurrentAllianceColor = OFF;

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

  ShooterRunState lastShooterRunState = ShooterRunState.NONE;

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
      Climber climber;

      public RGB(Shooter shooter, Intake intake, Climber climber) {
      // PWM port 9
      // Must be a PWM header, not MXP or DIO
      this.shooter = shooter;
      this.intake = intake;
      this.climber = climber;

      CurrentAllianceColor = getAllianceColor();

      m_led.setLength(ledBuffer.getLength());

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

      setBufferDirty();
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

      setBufferDirty();
    }


  public Color getAllianceColor() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent())
    {
      Alliance a = alliance.get();
      if (a == Alliance.Red) { return RED; } 
      else if (a == Alliance.Blue) { return BLUE; }
      else { return OFF; }
    }

    return OFF;
  }

  public void rainbowRGB()
  {
      for (int i = 0; i < ledBuffer.getLength(); i ++) {
        ledBuffer.setHSV(i, periodicCalls + i % 180, 255, (int)(255 * rgbStrength));
      }

      setBufferDirty();
  }

  public void allRandom()
  {
      for (int i = 0; i < ledBuffer.getLength(); i ++) {
        ledBuffer.setRGB(i, (int)(Math.random() * 255 * rgbStrength), (int)(Math.random() * 255 * rgbStrength), (int)(Math.random() * 255 * rgbStrength));
      }

      setBufferDirty();
  }

  void SetAscendingOrderRGB()
  {
      for (int i = 0; i < ledBuffer.getLength(); i ++) {
        int val = i;
        ledBuffer.setRGB(i, val, val, val);
      }

      setBufferDirty();
  }

  public void setBufferDirty() { bufferDirty = true; }

  @Override
  public void periodic() {
    periodicCalls++;
    updateFlashCommands();

    if (shouldUseFlash()) {
      setSolidRGBColor(currentFlashColor);
    }
    else {
      Color backgroundColor = OFF;

      if (!DriverStation.isTeleop() || !DriverStation.isEnabled())
      {
        backgroundColor = hsvToRgb(periodicCalls * 1.5f % 360, 1, 1);
      }

      // if in teleop, check for end of match and start blinking when match time reaches certain time
      if (DriverStation.isTeleop())
      {
        double matchTime = DriverStation.getMatchTime();

        if (!startedEndOfMatchFlash && matchTime > 0 && matchTime < endOfMatchStartFlashTime)
        {
          flashCommands.add(new RgbFlashCommand(YELLOW, 30, 20));
          startedEndOfMatchFlash = true;
        }
      }

      // if pick up note, flash orange
      if (!limitSwitchLastPeriodic && intake.limitSwitchPressed()) { flashCommands.add(new RgbFlashCommand(ORANGE, 2, 10)); }

      // flow statement for rgb colors
      if (Vision.usingLimelight) { backgroundColor = GREEN; }
      else if (intake.getCurrentPivotState() == IntakePivotState.OUT || intake.getCurrentPivotAngle() < IntakeConstants.INTAKE_UP_POS) { backgroundColor = CurrentAllianceColor; }

      setSolidRGBColor(backgroundColor);

      // set shooter progress rgb
      ShooterRunState currentShooterRunState =  shooter.getCurrentRunState();
      if (currentShooterRunState == ShooterRunState.AMP || currentShooterRunState == ShooterRunState.SHOOT) { lastShooterRunState = currentShooterRunState; }

      double targetSpeed = lastShooterRunState == ShooterRunState.AMP ? (ShooterConstants.LEFT_AMP_SPEED + ShooterConstants.RIGHT_AMP_SPEED) / 2 : (ShooterConstants.LEFT_SHOOTER_SPEED + ShooterConstants.RIGHT_SHOOTER_SPEED) / 2;
      double shooterRPMPercentOfMax = shooter.getShooterRPM() / targetSpeed;
      if (shooterRPMPercentOfMax > 0) {
        Color fillColor = lerpColor(DARK_RED, DARK_GREEN, shooterRPMPercentOfMax * shooterRPMPercentOfMax);
        SHOOTER_RIGHT_ZONE.setProgressColor(shooterRPMPercentOfMax, fillColor, backgroundColor);
        SHOOTER_LEFT_ZONE.setProgressColor(shooterRPMPercentOfMax, fillColor, backgroundColor);
      }

      // set climber progress rgb
      double leftClimberPercent =  climber.leftClimberRotations() / ClimberConstants.CLIMBER_MAX_POS;
      double rightClimberPercent = climber.rightClimberRotations() / ClimberConstants.CLIMBER_MAX_POS;
      if (leftClimberPercent > 0.05) { CLIMBER_LEFT_ZONE.setProgressColor(leftClimberPercent, PURPLE, backgroundColor); }
      if (rightClimberPercent > 0.05) { CLIMBER_RIGHT_ZONE.setProgressColor(rightClimberPercent, PURPLE, backgroundColor); }
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

  public Color lerpColor(Color startColor, Color endColor, double t)
  {
    return new Color (startColor.red + ((endColor.red - startColor.red) * t), startColor.green + ((endColor.green - startColor.green) * t), startColor.blue + ((endColor.blue - startColor.blue) * t));
  }

  void updateFlashCommands()
  {
    for (int i = flashCommands.size() - 1; i > -1; i--)
    {
      RgbFlashCommand currentCommand = flashCommands.get(i);
      currentCommand.periodicUpdate();

      if (currentCommand.shouldDelete()) { flashCommands.remove(i); }
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

    currentFlashColor = OFF;
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