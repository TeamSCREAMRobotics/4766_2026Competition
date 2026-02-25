package frc.robot.subsystems.ShooterSubFolder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import frc.robot.Constants.ShooterConstants;

public class LFlywheel extends TalonFXSubsystem {
  TalonFX lflywheel = new TalonFX(ShooterConstants.LshooterMoterID);

  public LFlywheel(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public double getvelocity() {
    return lflywheel.getVelocity().getValueAsDouble();
  }
}
