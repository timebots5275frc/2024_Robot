// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants.AprilTagData;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.Vision;
import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;

public class AutoVisionDetectAction extends Command {
  SwerveDrive swerveDrive;
  Shooter shooter;
  Vision vision;

  Command subcommand;
  boolean finished = false;

  public AutoVisionDetectAction(SwerveDrive swerveDrive, Shooter shooter, Vision vision) {
    this.swerveDrive = swerveDrive;
    this.shooter = shooter;
    this.vision = vision;
  }

  @Override
  public void initialize() 
  {
    int currentAprilTagID = vision.AprilTagID();

    if (currentAprilTagID != -1)
    {
      AprilTagData currenAprilTag = AprilTagData.getTag(currentAprilTagID);

      // MAY NOT ALWAYS RETURN AN ALLIANCE
      Optional<DriverStation.Alliance> optionalAllianceRequest = DriverStation.getAlliance();

      if (!optionalAllianceRequest.isPresent() || (optionalAllianceRequest.isPresent() && currenAprilTag.alliance == optionalAllianceRequest.get()))
      {
        if (AprilTagData.isSpeakerTag(currenAprilTag))
        {
          subcommand = new AutoVisionSpeakerShoot(swerveDrive, shooter, vision);
        }
        else if(AprilTagData.isAmpTag(currenAprilTag))
        {
          // Do silly stuff
        }
      }

      subcommand.schedule();
    }

    finished = true;
  }

  @Override
  public void end(boolean interrupted)
  {
    if (subcommand != null && !subcommand.isFinished()) { subcommand.cancel(); }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
