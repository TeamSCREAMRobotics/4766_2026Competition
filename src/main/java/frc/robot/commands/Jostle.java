// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Jostle extends Command {
  Intake m_Intake;
  Timer timer;
  int timerTest;
  Debouncer jostlDebouncer;

  /** Creates a new Jostle. */
  public Jostle(Intake intake) {
    m_Intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
    timerTest = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("timer", Timer.getTimestamp());
    // if(Timer.getTimestamp() > 0.25){
    // m_Intake.IntakeGoToSetpoint(IntakeConstants.intakePivotDownSetpoint);
    //   if(Timer.getTimestamp() > 0.5){
    //     timer = new Timer();
    //     timer.start();

    //   }
    // }
    // else{
    //   m_Intake.IntakeGoToSetpoint(IntakeConstants.intakeAgitateSetpoint);
    // }
    m_Intake.runIntake(6);
    if (timerTest > 24) {
      m_Intake.IntakeGoToSetpoint(IntakeConstants.intakePivotDownSetpoint);
      timerTest++;
      if (timerTest > 50) {
        timerTest = 0;
      }
    } else {
      m_Intake.IntakeGoToSetpoint(IntakeConstants.intakeAgitateSetpoint);
      timerTest++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.IntakeGoToSetpoint(IntakeConstants.intakePivotDownSetpoint);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
