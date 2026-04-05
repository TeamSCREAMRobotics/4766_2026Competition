// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Dashboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.ShooterSubFolder.Flywheel;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {

  Indexer s_Indexer;
  Flywheel s_Flywheel;

  public static DoubleSupplier desiredvelocity;

  /** Creates a new Shooter. */
  public Shoot(Flywheel flywheel, Indexer s_Indexer, DoubleSupplier Desiredvelocity) {

    this.s_Flywheel = flywheel;
    this.s_Indexer = s_Indexer;
    desiredvelocity = Desiredvelocity;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Desired Velocity", desiredvelocity.getAsDouble());
    if (s_Flywheel.getVelocity() >= Dashboard.flywheelVelocity.get() - 2.0
        && s_Flywheel.getVelocity() <= Dashboard.flywheelVelocity.get() + 2.0)
    /*s_Flywheel.getVelocity() >= desiredvelocity.getAsDouble() - 0.5
    && s_Flywheel.getVelocity() <= desiredvelocity.getAsDouble() + 0.5)*/ {
      s_Indexer.runIndexer(12);
    } else {
      s_Indexer.runIndexer(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Indexer.runIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
