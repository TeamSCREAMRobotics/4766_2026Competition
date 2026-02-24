// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunClimber extends Command {
  Climber m_Climber;
  Double m_setpoint;

  /** Creates a new RunClimber. */
  public RunClimber(Climber climber, Double setpoint) {
    m_Climber = climber;
    m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climber.climberGoToSetpoint(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    DogLog.log("Climber Finished", m_Climber.climberIsFinished(m_setpoint));
    return m_Climber.climberIsFinished(m_setpoint);
  }
}
