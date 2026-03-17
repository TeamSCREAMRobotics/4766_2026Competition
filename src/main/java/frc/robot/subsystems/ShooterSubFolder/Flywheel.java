package frc.robot.subsystems.ShooterSubFolder;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import frc.robot.constants.Constants.ShooterConstants;

public class Flywheel extends TalonFXSubsystem {

  private VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0);

  public Flywheel(TalonFXSubsystemConfiguration config) {

    super(config);
  }

  public void setTargetVelocityTorqueCurrent(double velocity, double torqueFeedForward) {
    super.setpoint = velocity;
    super.inVelocityMode = true;
    setMaster(velocityTorqueCurrentFOC.withVelocity(velocity).withFeedForward(torqueFeedForward));
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
