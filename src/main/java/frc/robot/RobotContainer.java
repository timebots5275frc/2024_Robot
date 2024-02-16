// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoIntake;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.IntakeRunCommand;
import frc.robot.commands.ResetClimberCommand;
import frc.robot.commands.ShooterPivotCommand;
import frc.robot.commands.ShooterRunCommand;
//import frc.robot.commands.IntakeRunCommand;
import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber.ClimberMode;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Input.Input;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Intake.IntakeRunState;
import frc.robot.subsystems.Shooter.ShooterPivotState;
import frc.robot.subsystems.Shooter.ShooterRunState;
import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  SwerveDrive swerveDrive;
  Shooter shooter;
  Intake intake;
  Vision vision;
  Climber climber;

  Joystick driveStick;
  //GenericHID buttonBoard;

  Input input;

  TeleopJoystickDrive joyDrive;

  //AutoIntake autoIntake;
  
  //private Climber climber;

  public static boolean intakeCanMove;

  public static boolean shooterReadyToShoot;

  SequentialCommandGroup autoCommands;

  public RobotContainer() {
    swerveDrive = new SwerveDrive();
    shooter = new Shooter();
    intake = new Intake();
    climber = new Climber();

    driveStick = new Joystick(0);
    //buttonBoard = new GenericHID(1);
    input = new Input(driveStick);
    vision = new Vision();
    //climber = new Climber();
    joyDrive = new TeleopJoystickDrive(swerveDrive, driveStick, input, true);

    intakeCanMove = true;
    shooterReadyToShoot = false;

    configureBindings();
  }

  private void configureBindings() {
    swerveDrive.setDefaultCommand(joyDrive);
    new JoystickButton(driveStick, 8).onTrue(new InstantCommand(swerveDrive::resetPigeon, swerveDrive));

    //.onTrue() calls command once per button press
    //.whileTrue() calls command while button is held or until command finishes
    //.toggleOnTrue() makes a toggle which runs when pressed and then stops when presse again


    // new JoystickButton(driveStick, 6).onTrue(new IntakePivotCommand(intake, IntakePivotState.IN));
    // new JoystickButton(driveStick, 5).onTrue(new IntakePivotCommand(intake, IntakePivotState.OUT));

    // new JoystickButton(driveStick, 1).onTrue(new ParallelCommandGroup(new ShooterPivotCommand(shooter, ShooterPivotState.DEFAULT_SHOOT), new ShooterRunCommand(shooter, ShooterRunState.SHOOT)));
    // new JoystickButton(driveStick, 2).onTrue(new ParallelCommandGroup(new ShooterPivotCommand(shooter, ShooterPivotState.START), new ShooterRunCommand(shooter, ShooterRunState.NONE)));


    // new JoystickButton(driveStick, 9).onTrue(new ClimberCommand(climber, ClimberMode.EXTEND));
    // new JoystickButton(driveStick, 10).onTrue(new ClimberCommand(climber, ClimberMode.RETRACT));
    // new JoystickButton(driveStick, 8).whileTrue(new ClimberCommand(climber, ClimberMode.RESET));
  
    // Test Buttons

    new JoystickButton(driveStick, 3).onTrue(new IntakePivotCommand(intake, IntakePivotState.IN));
    new JoystickButton(driveStick, 4).onTrue(new IntakePivotCommand(intake, IntakePivotState.OUT));

    new JoystickButton(driveStick, 5).onTrue(new IntakeRunCommand(intake, IntakeRunState.FORWARD));
    new JoystickButton(driveStick, 6).onTrue(new IntakeRunCommand(intake, IntakeRunState.REVERSE));
    new JoystickButton(driveStick, 2).onTrue(new IntakeRunCommand(intake, IntakeRunState.NONE));

    new JoystickButton(driveStick,11).onTrue(new ShooterRunCommand(shooter, ShooterRunState.NONE));
    new JoystickButton(driveStick, 12).onTrue(new ShooterRunCommand(shooter, ShooterRunState.SHOOT));

    new JoystickButton(driveStick, 0).whileTrue(new ClimberCommand(climber, ClimberMode.EXTEND));
    new JoystickButton(driveStick, 0).whileTrue(new ClimberCommand(climber, ClimberMode.RETRACT));
    new JoystickButton(driveStick, 0).whileTrue(new ResetClimberCommand(climber));

    // new JoystickButton(driveStick, 5).onTrue(new ShooterPivotCommand(shooter, ShooterPivotState.DEFAULT_SHOOT));
    // new JoystickButton(driveStick, 6).onTrue(new ShooterPivotCommand(shooter, ShooterPivotState.AMP));


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
