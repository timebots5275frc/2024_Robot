// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoIntake;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Input.Input;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  SwerveDrive drive;
  Shooter shooter;
  Intake intake;

  Joystick driveStick;
  GenericHID buttonBoard;

  Input input;

  TeleopJoystickDrive joyDrive;

  AutoIntake autoIntake;

  IntakeCommand intakeIdle, intakeRest, intakeIntake, intakeEject, intakeReady, intakeFeed;

  ShooterCommand shooterStart, shooterIdle, shooterFire, shooterAmp, shooterTest;
  
  SequentialCommandGroup autoCommands;

  public RobotContainer() {
    drive = new SwerveDrive();
    shooter = new Shooter();
    intake = new Intake(shooter);

    driveStick = new Joystick(0);
    buttonBoard = new GenericHID(1);
    input = new Input(driveStick);

    joyDrive = new TeleopJoystickDrive(drive, driveStick, input, true);
    intakeIdle = new IntakeCommand(intake, IntakeState.IDLE);
    intakeRest = new IntakeCommand(intake, IntakeState.REST);
    intakeIntake = new IntakeCommand(intake, IntakeState.INTAKE);
    intakeEject = new IntakeCommand(intake, IntakeState.EJECT);
    intakeReady = new IntakeCommand(intake, IntakeState.READY_TO_FEED);
    intakeFeed = new IntakeCommand(intake, IntakeState.FEED_SHOOTER);

    autoIntake = new AutoIntake(intake);

    shooterStart = new ShooterCommand(shooter, ShooterState.START);
    shooterIdle = new ShooterCommand(shooter, ShooterState.IDLE);
    shooterFire = new ShooterCommand(shooter, ShooterState.VISION_SHOOT);
    shooterAmp = new ShooterCommand(shooter, ShooterState.AMP);
    shooterTest = new ShooterCommand(shooter, ShooterState.TEST);

    configureBindings();

    //autoCommands = new SequentialCommandGroup(null);
  }

  private void configureBindings() {
    //intake.setDefaultCommand(autoIntake);

    //.onTrue() calls command once per button press
    //.whileTrue() calls command while button is held or until command finishes
    //.toggleOnTrue() makes a toggle which runs when pressed and then stops when presse again

    new JoystickButton(buttonBoard, 1).onTrue(new RepeatCommand(shooterTest));
    new JoystickButton(buttonBoard, 2).onTrue(new RepeatCommand(shooterStart));
    new JoystickButton(buttonBoard, 3).onTrue(new RepeatCommand(shooterIdle));

    new JoystickButton(buttonBoard, 4).onTrue(new RepeatCommand(intakeIdle));
    new JoystickButton(buttonBoard, 5).onTrue(new RepeatCommand(intakeIntake).until(intake.autoIntake));
    new JoystickButton(buttonBoard, 6).onTrue(new RepeatCommand(intakeReady));
    new JoystickButton(buttonBoard, 7).onTrue(new RepeatCommand(intakeFeed));
    
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
