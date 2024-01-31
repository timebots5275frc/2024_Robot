// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Input;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.CustomTypes.Math.Vector2;

public class Input extends SubsystemBase {
  Joystick driveJoystick;

  Vector2 rawJoystickInput = Vector2.zero;
  double rawJoystickTwist = 0;

  Vector2 joystickInput = Vector2.zero;
  double joystickTwist = 0;

  public BooleanSupplier receivingJoystickInput = new BooleanSupplier() {
    public boolean getAsBoolean() { return joystickInput.x != 0 || joystickInput.y != 0; }
  };

  public Input(Joystick driveJoystick) {
    this.driveJoystick = driveJoystick;
  }

  @Override
  public void periodic() {
    getRawJoystickInput();
    calculateJoystickInput();
  }

  void getRawJoystickInput()
  {
    rawJoystickInput = new Vector2(-driveJoystick.getY(), -driveJoystick.getX());
    rawJoystickTwist = -driveJoystick.getTwist();
  }

  void calculateJoystickInput()
  {
    joystickInput = new Vector2(calculateInputWithDeadzone(rawJoystickInput.x, Constants.ControllerConstants.DEADZONE_DRIVE), calculateInputWithDeadzone(rawJoystickInput.y, Constants.ControllerConstants.DEADZONE_DRIVE));
    joystickTwist = calculateInputWithDeadzone(rawJoystickTwist, Constants.ControllerConstants.DEADZONE_STEER);
  }

  public Vector2 JoystickInput() { return joystickInput; }
  public double JoystickTwist() { return joystickTwist; }

  public double calculateInputWithDeadzone(double input, double deadZone) {
    if (Math.abs(input) < deadZone) {
        return 0;
    }

    if (input > 0) {
        return (input - deadZone) / (1 - deadZone);
    } else if (input < 0) {
        return (input + deadZone) / (1 - deadZone);
    }
    return 0;
}
}
