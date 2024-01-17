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
  private int toggle;

  public TestIntakeCommand(TestIntake intake, Joystick stick) {
    // Use addRequirements() here to declare subsystem dependencies.
    testIntake = intake;
    joy = stick;
    toggle = 0;
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
      toggle = 1;
    }
    else if (joy.getRawButtonPressed(3)) {
      toggle = 2;
    }
    else if (joy.getRawButtonPressed(5)) {
      toggle = 0;
    }
    if (toggle == 1) {
      testIntake.runIntake(1);
    } else if (toggle == 2) {
      testIntake.runIntake(-1);
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
