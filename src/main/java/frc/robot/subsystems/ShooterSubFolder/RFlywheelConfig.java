package frc.robot.subsystems.ShooterSubFolder;

import com.ctre.phoenix6.signals.InvertedValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;
import frc.robot.Constants.ShooterConstants;

public class RFlywheelConfig {
  public static final TalonFXSubsystemConfiguration RFLYWHEEL_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    RFLYWHEEL_CONFIG.name = "LFlywheel";

    RFLYWHEEL_CONFIG.codeEnabled = true;
    RFLYWHEEL_CONFIG.logTelemetry = false;
    RFLYWHEEL_CONFIG.debugMode = false;

    RFLYWHEEL_CONFIG.masterConstants =
        new TalonFXConstants(
            new CANDevice(ShooterConstants.RshooterMotorID),
            InvertedValue.CounterClockwise_Positive);

    RFLYWHEEL_CONFIG.slot0 =
        new ScreamPIDConstants(0.1, 0, 0)
            .getSlot0Configs(new FeedforwardConstants(0.11635, 0.17347, 0.0, 0.0));

    RFLYWHEEL_CONFIG.enableStatorCurrentLimit = true;
    RFLYWHEEL_CONFIG.statorCurrentLimit = 20;
  }
}
