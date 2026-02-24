// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Agitator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Kick extends Command {
  AgitatorSub s_agitator;
  double kickerVoltage;

  /** Creates a new Kick. */
  public Kick(AgitatorSub agitator, double kickerVoltage) {
    s_agitator = agitator;
    this.kickerVoltage = kickerVoltage;

    addRequirements(s_agitator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_agitator.RunKicker(kickerVoltage);
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
