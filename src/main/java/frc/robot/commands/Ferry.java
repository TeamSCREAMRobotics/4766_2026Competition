// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.teamscreamrobotics.vision.LimelightHelpers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.AgitatorSub;
import frc.robot.subsystems.ShooterSubFolder.LFlywheel;
import frc.robot.subsystems.ShooterSubFolder.RFlywheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Ferry extends Command {
  AgitatorSub s_Agitator;

  RFlywheel s_RFlywheel;
  LFlywheel s_LFlywheel;

  public static double desiredferryvelocity;
  /** Creates a new Ferry. */
  public Ferry(LFlywheel lFlywheel, RFlywheel rFlywheel, AgitatorSub agitator, double Desiredferryvelocity) {
           

    s_LFlywheel = lFlywheel;
    s_RFlywheel = rFlywheel;
    desiredferryvelocity = Desiredferryvelocity;

    s_Agitator = agitator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(agitator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
   @Override
  public void initialize() {
    desiredferryvelocity =
        ShooterConstants.FERRY_VELOCITY_MAP.get(LimelightHelpers.getTA("limelight-shooter"));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_LFlywheel.getvelocity() >= desiredferryvelocity - 1
        && s_LFlywheel.getvelocity() <= desiredferryvelocity + 1
        && s_RFlywheel.getvelocity() >= desiredferryvelocity - 1
        && s_RFlywheel.getvelocity() <= desiredferryvelocity + 1) {
      s_Agitator.RunAgitatorAndKicker(10, 12);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Agitator.RunAgitatorAndKicker(0, 0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
