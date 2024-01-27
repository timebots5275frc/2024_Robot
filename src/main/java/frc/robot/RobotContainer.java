// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoIntake;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  Shooter shooter;
  Intake intake;

  AutoIntake autoIntake = new AutoIntake(intake, shooter);

  IntakeCommand intakeIdle, intakeRest, intakeIntake, intakeEject, intakeReady, intakeFeed;

  ShooterCommand shooterStart, shooterIdle, shooterFire, shooterAmp, shooterTest;

  public RobotContainer() {
    shooter = new Shooter();
    intake = new Intake(shooter);

    intakeIdle = new IntakeCommand(intake, IntakeState.IDLE);
    intakeRest = new IntakeCommand(intake, IntakeState.REST);
    intakeIntake = new IntakeCommand(intake, IntakeState.INTAKE);
    intakeEject = new IntakeCommand(intake, IntakeState.EJECT);
    intakeReady = new IntakeCommand(intake, IntakeState.READY_TO_FEED);
    intakeFeed = new IntakeCommand(intake, IntakeState.FEED_SHOOTER);

    shooterStart = new ShooterCommand(shooter, ShooterState.START);
    shooterIdle = new ShooterCommand(shooter, ShooterState.IDLE);
    shooterFire = new ShooterCommand(shooter, ShooterState.VISION_SHOOT);
    shooterAmp = new ShooterCommand(shooter, ShooterState.AMP);
    shooterTest = new ShooterCommand(shooter, ShooterState.TEST);

    configureBindings();
  }

  private void configureBindings() {

    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
