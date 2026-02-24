// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSub;
import frc.robot.subsystems.ShooterSubFolder.LFlywheel;
import frc.robot.subsystems.ShooterSubFolder.RFlywheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {

  RFlywheel s_RFlywheel;
  AgitatorSub s_Agitator;
  LFlywheel s_LFlywheel;
  double lvelocity;
  double rvelocity;

  /** Creates a new Shooter. */
  public Shoot(
      LFlywheel lFlywheel, RFlywheel rFlywheel, AgitatorSub agitator, double lv, double rv) {

    s_LFlywheel = lFlywheel;
    s_RFlywheel = rFlywheel;
    s_Agitator = agitator;
    lvelocity = lv;
    rvelocity = rv;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(agitator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // s_Shooter.runShooter(lvoltage, rvoltage);
    if (s_LFlywheel.getvelocity() >= lvelocity - 2
        && s_LFlywheel.getvelocity() <= lvelocity + 2
        && s_RFlywheel.getvelocity() >= rvelocity - 2
        && s_RFlywheel.getvelocity() <= rvelocity + 2) {
      s_Agitator.RunAgitatorAndKicker(3, 3);

      // (LeftShooter, RightShooter)
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
