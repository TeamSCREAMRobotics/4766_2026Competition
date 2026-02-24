// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Agitator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Agitate extends Command {
  AgitatorSub s_agitator;
  double agitatorVoltage;

  /** Creates a new Agitate. */
  public Agitate(AgitatorSub agitator, double agitatorVoltage) {
    s_agitator = agitator;
    this.agitatorVoltage = agitatorVoltage;

    addRequirements(agitator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_agitator.RunAgitator(agitatorVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //  s_agitator.RunAgitator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
