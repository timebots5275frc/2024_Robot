// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestIntake;

public class TestIntakeCommand extends Command {
  /** Creates a new TestIntakeCommand. */

  private TestIntake testIntake;
  private Joystick joy;
  private boolean toggle;

  public TestIntakeCommand(TestIntake intake, Joystick stick) {
    // Use addRequirements() here to declare subsystem dependencies.
    testIntake = intake;
    joy = stick;
    toggle = false;
    addRequirements(testIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (joy.getRawButtonPressed(4)) {
      toggle = !toggle;
    }

    if (toggle) {
      testIntake.runIntake();
    } else {
      testIntake.stopIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
