// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.VisionConstants.AprilTagData;
import frc.robot.CustomTypes.ManagerCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.Vision;
import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoVisionDetectAction extends ManagerCommand {
  SwerveDrive swerveDrive;
  Shooter shooter;
  Vision vision;
  Intake intake;

  boolean finished = false;

  public AutoVisionDetectAction(SwerveDrive swerveDrive, Shooter shooter, Vision vision, Intake intake) {
    this.swerveDrive = swerveDrive;
    this.shooter = shooter;
    this.vision = vision;
    this.intake = intake;
  }

  @Override
  public void initialize() 
  {
    int currentAprilTagID = vision.AprilTagID();
    Command subCommand = null;

    if (currentAprilTagID != -1)
    {
      AprilTagData currenAprilTag = AprilTagData.getTag(currentAprilTagID);

      // MAY NOT ALWAYS RETURN AN ALLIANCE
      Optional<DriverStation.Alliance> optionalAllianceRequest = DriverStation.getAlliance();

      if (!optionalAllianceRequest.isPresent() || (optionalAllianceRequest.isPresent() && currenAprilTag.alliance == optionalAllianceRequest.get()))
      {
        if (AprilTagData.isSpeakerTag(currenAprilTag))
        {
          subCommand = new AutoVisionSpeakerShoot(swerveDrive, shooter, vision, intake);
        }
        else if(AprilTagData.isAmpTag(currenAprilTag))
        {
          // Do silly stuff
        }
      }

      if (subCommand != null) { scheduleSubcommand(subCommand); }
    }

    finished = true;
  }

  @Override
  public boolean isFinished() {
    return finished || subCommandFinished();
  }
}
