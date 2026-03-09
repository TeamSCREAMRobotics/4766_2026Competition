package frc.robot.subsystems.ShooterSubFolder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import frc.robot.Constants.ShooterConstants;

public class RFlywheel extends TalonFXSubsystem {
  TalonFX rflywheel = new TalonFX(ShooterConstants.RshooterMotorID);

  public RFlywheel(TalonFXSubsystemConfiguration config) {
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
