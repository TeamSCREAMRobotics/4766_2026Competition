// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class QuickJostle extends Command {
  Intake m_Intake;
  int timer;
  int intTimer;

  /** Creates a new Jostle. */
  public QuickJostle(Intake intake) {
    m_Intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    intTimer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("timer", timer);
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
    m_Intake.runIntake(8);
    if (timer > 24) {
      m_Intake.IntakeGoToSetpoint(IntakeConstants.intakeAgitateLowSetpoint);
      timer++;
      if (timer > 50) {
        timer = 0;
      }
    } else {
      m_Intake.IntakeGoToSetpoint(IntakeConstants.intakeAgitateHighSetpoint);
      timer++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.IntakeGoToSetpoint(IntakeConstants.intakePivotDownSetpoint);
    m_Intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
