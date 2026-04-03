package frc.robot.subsystems.ShooterSubFolder;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;

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

  public double getflywheelvoltage() {
    return master.getMotorVoltage().getValueAsDouble();
  }
}
