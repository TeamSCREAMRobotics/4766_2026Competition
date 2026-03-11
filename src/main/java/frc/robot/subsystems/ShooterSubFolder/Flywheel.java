package frc.robot.subsystems.ShooterSubFolder;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import frc.robot.constants.Constants.ShooterConstants;

public class Flywheel extends TalonFXSubsystem {
  TalonFX rflywheel = new TalonFX(ShooterConstants.RshooterMotorID);
  TalonFX lflywheel = new TalonFX(ShooterConstants.LshooterMoterID);

  public Flywheel(TalonFXSubsystemConfiguration config) {
    
    super(config);
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public double getvelocity() {
    return rflywheel.getVelocity().getValueAsDouble();
  }
}
