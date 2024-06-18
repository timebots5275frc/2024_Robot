// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.VisionConstants;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoOdometryDrive;
import frc.robot.commands.AutoVisionAmpShoot;
import frc.robot.commands.AutoVisionDrive;
import frc.robot.commands.AutoVisionSpeakerShoot;
import frc.robot.commands.AutoVisionStageShoot;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.IntakeRunCommand;
import frc.robot.commands.ResetClimberCommand;
import frc.robot.commands.RotateAndLogCommand;
import frc.robot.commands.ShooterPivotCommand;
import frc.robot.commands.ShooterRunCommand;
import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.commands.ToggleAutoTarget;
import frc.robot.commands.UseLimelightCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RGB;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber.ClimberMode;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Input.Input;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Intake.IntakeRunState;
import frc.robot.subsystems.Shooter.ShooterPivotState;
import frc.robot.subsystems.Shooter.ShooterRunState;
import frc.robot.subsystems.Vision.AutoTargetStateManager;
import frc.robot.subsystems.Vision.Vision;

import java.util.function.BooleanSupplier;

import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


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
  RGB rgb;

  GenericHID buttonBoard;
  GenericHID driveInput;

  Input input;

  TeleopJoystickDrive joyDrive;

  SequentialCommandGroup autoCommands;

  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    boolean usingJoystick = false;
    swerveDrive = new SwerveDrive();
    shooter = new Shooter();
    intake = new Intake();
    climber = new Climber();
    rgb = new RGB(shooter, intake, climber);

    buttonBoard = new GenericHID(1);
    if (usingJoystick) {Joystick driveStick = new Joystick(0); driveInput = driveStick; } else {XboxController controller = new XboxController(0); driveInput = controller; }
    input = new Input(driveInput);
    vision = new Vision();
    joyDrive = new TeleopJoystickDrive(swerveDrive, input, true);

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Vision Shoot Turn", AutoVisionSpeakerShoot.ShootAndStopCommand(shooter, swerveDrive, vision, intake));
    autoChooser.addOption("Vision Shoot No Turn", AutoVisionSpeakerShoot.ShootVisionCommandAutoFirstShot(shooter, intake, swerveDrive));
    autoChooser.addOption("4-Note LMR", AutoCommands.leftMiddleRightAutoCommand(swerveDrive, vision, shooter, intake));
    autoChooser.addOption("4-Note RML", AutoCommands.rightMiddleLeftAutoCommand(swerveDrive, vision, shooter, intake));
    autoChooser.addOption("2-Note L", AutoCommands.shootLeftShootAutoCommand(swerveDrive, vision, shooter, intake));
    autoChooser.addOption("2-Note R", AutoCommands.shootRightShootAutoCommand(swerveDrive, vision, shooter, intake));
    autoChooser.addOption("2-Note M", AutoCommands.shootMiddleShootAutoCommand(swerveDrive, vision, shooter, intake));
    autoChooser.addOption("Blue Taxi No Turn", AutoCommands.blueSideOneNoteTaxiNoTurn(swerveDrive, vision, shooter, intake));
    autoChooser.addOption("Red Taxi No Turn", AutoCommands.redSideOneNoteTaxiNoTurn(swerveDrive, vision, shooter, intake));
    autoChooser.addOption("Blue Taxi Turn", AutoCommands.blueSideOneNoteTaxiTurn(swerveDrive, vision, shooter, intake));
    autoChooser.addOption("Red Taxi Turn", AutoCommands.redSideOneNoteTaxiTurn(swerveDrive, vision, shooter, intake));
    SmartDashboard.putData(autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    swerveDrive.setDefaultCommand(joyDrive); 
    if (input.usingJoystick) {
      new JoystickButton(driveInput, 8).onTrue(new InstantCommand(swerveDrive::resetPigeon, swerveDrive));
      new JoystickButton(driveInput, 12).whileTrue(new ResetClimberCommand(climber));

      //new JoystickButton(driveStick, 9).onTrue(new UseLimelightCommand(true));
      new JoystickButton(driveInput, 10).onTrue(new UseLimelightCommand(false));
      new JoystickButton(driveInput, 9).onTrue(new IntakeRunCommand(intake, IntakeRunState.INTAKE).until(intake.LimitSwitchIsPressed));

      new JoystickButton(driveInput, 1).onTrue(AutoVisionSpeakerShoot.ShootAndStopCommand(shooter, swerveDrive, vision, intake));
      new JoystickButton(driveInput, 11).onTrue(AutoVisionAmpShoot.GetCommand(swerveDrive, vision, shooter, intake).until(input.receivingJoystickInput));
      //new JoystickButton(driveStick, 11).onTrue(AutoVisionStageShoot.getCommand(shooter, intake, vision, swerveDrive).until(input.receivingJoystickInput));


      //new JoystickButton(driveStick, 2).onTrue(new RotateAndLogCommand(vision, swerveDrive));

      new JoystickButton(driveInput, 4).onTrue(new IntakeRunCommand(intake, IntakeRunState.NONE));


      new JoystickButton(driveInput, 2).onTrue(new ToggleAutoTarget());

      new JoystickButton(driveInput, 5).onTrue(new SequentialCommandGroup(new IntakePivotCommand(intake, IntakePivotState.IN), new WaitCommand(0.5), new IntakeRunCommand(intake, IntakeRunState.NONE)));
      new JoystickButton(driveInput, 6).onTrue(new SequentialCommandGroup(new IntakePivotCommand(intake, IntakePivotState.OUT), new WaitCommand(0.5), new IntakeRunCommand(intake, IntakeRunState.INTAKE)));

    } else {
      new Trigger(((XboxController) driveInput).getRightTriggerAxis() > 0.1)
    }

    new JoystickButton(buttonBoard, 6).onTrue(new SequentialCommandGroup(new IntakeRunCommand(intake, IntakeRunState.OUTTAKE), new WaitCommand(0.5), new IntakeRunCommand(intake, IntakeRunState.NONE)));
    new JoystickButton(buttonBoard,9).onTrue(new ShooterRunCommand(shooter, ShooterRunState.NONE));
    new JoystickButton(buttonBoard, 11).onTrue(new ShooterRunCommand(shooter, ShooterRunState.SHOOT));
    new JoystickButton(buttonBoard, 7).onTrue(new ShooterRunCommand(shooter, ShooterRunState.AMP));


    new JoystickButton(buttonBoard, 5).onTrue(new ShooterPivotCommand(shooter, ShooterPivotState.SHOOTER_45));
    new JoystickButton(buttonBoard, 3).onTrue(new ShooterPivotCommand(shooter, ShooterPivotState.DEFAULT_SHOOT));
    new JoystickButton(buttonBoard, 8).onTrue(new SequentialCommandGroup(new IntakePivotCommand(intake, IntakePivotState.OUT), new WaitCommand(0.5), new ShooterPivotCommand(shooter, ShooterPivotState.CLIMBING_POS)));
    new JoystickButton(buttonBoard, 4).onTrue(new ShooterPivotCommand(shooter, ShooterPivotState.AMP));

    new JoystickButton(buttonBoard, 12).whileTrue(new ClimberCommand(climber, ClimberMode.EXTEND));
    new JoystickButton(buttonBoard, 10).whileTrue(new ClimberCommand(climber, ClimberMode.RETRACT));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
