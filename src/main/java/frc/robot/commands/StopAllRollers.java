// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.ShooterSubFolder.Flywheel;

public class StopAllRollers extends Command {

  private Flywheel m_Flywheel;
  private Indexer m_Indexer;

  public StopAllRollers(Flywheel flywheel, Indexer indexer) {
    m_Flywheel = flywheel;
    m_Indexer = indexer;

    addRequirements(m_Flywheel);
    addRequirements(m_Indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // m_Flywheel.setSetpointVelocity(0);
    m_Indexer.runIndexer(0);
  }

  @Override
  public boolean isFinished() {
    return !Indexer.getShouldBeEnabled();
  }
}
