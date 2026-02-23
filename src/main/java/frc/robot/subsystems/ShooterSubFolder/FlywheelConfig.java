package frc.robot.subsystems.ShooterSubFolder;

import com.ctre.phoenix6.signals.InvertedValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;

public class FlywheelConfig {
  public static final TalonFXSubsystemConfiguration FLYWHEEL_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    FLYWHEEL_CONFIG.name = "Flywheel";

    FLYWHEEL_CONFIG.codeEnabled = true;
    FLYWHEEL_CONFIG.logTelemetry = false;
    FLYWHEEL_CONFIG.debugMode = false;

    FLYWHEEL_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(17), InvertedValue.Clockwise_Positive);

    FLYWHEEL_CONFIG.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(16), InvertedValue.CounterClockwise_Positive)
        };
    FLYWHEEL_CONFIG.slot0 =
        new ScreamPIDConstants(0.1, 0.0, 0.0)
            .getSlot0Configs(new FeedforwardConstants(0.11635, 0.17347, 0.0, 0.0));

    FLYWHEEL_CONFIG.enableSupplyCurrentLimit = true;
    FLYWHEEL_CONFIG.supplyCurrentLimit = 20;
  }
}
