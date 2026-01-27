// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class Constants {

  public class DriveTrainConstants {}

  public class VisionConstants {}

  public class IntakeConstants {}

  public class ShooterConstants {
    public static final int LshooterMoterID = 7; // testing on testbed
    public static final int MshooterMotorID = 0;
    public static final int RshooterMotorID = 0;
    public static final int shooterCANID = 0;

    public static final int pivotMotorID = 0;
    public static final int pivotCanID = 0;

    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kMagicAcceleration = 0.0;
    public static final double kMagicCruiseVelocity = 0.0;

    public static final InterpolatingDoubleTreeMap HOOD_MAP = new InterpolatingDoubleTreeMap();

    static {
      // placeholder values before testing
      // (distance to hub, hood angle)
      HOOD_MAP.put(1.0, 10.0);
      HOOD_MAP.put(2.0, 20.0);
      HOOD_MAP.put(3.0, 30.0);
      HOOD_MAP.put(4.0, 40.0);
    }

    public static final InterpolatingDoubleTreeMap LSHOOTER_VELOCITY_MAP = new InterpolatingDoubleTreeMap();

    static {
      // placeholder values before testing
      // (distance to hub, flywheel voltage)
      LSHOOTER_VELOCITY_MAP.put(1.0, 1.0);
      LSHOOTER_VELOCITY_MAP.put(2.0, 3.0);
      LSHOOTER_VELOCITY_MAP.put(3.0, 5.0);
      LSHOOTER_VELOCITY_MAP.put(4.0, 6.5);
    }

    public static final InterpolatingDoubleTreeMap MSHOOTER_VELOCITY_MAP = new InterpolatingDoubleTreeMap();

    static {
      // placeholder values before testing
      // (distance to hub, flywheel voltage)
      MSHOOTER_VELOCITY_MAP.put(1.0, 1.0);
      MSHOOTER_VELOCITY_MAP.put(2.0, 3.0);
      MSHOOTER_VELOCITY_MAP.put(3.0, 5.0);
      MSHOOTER_VELOCITY_MAP.put(4.0, 6.5);
    }

    public static final InterpolatingDoubleTreeMap RSHOOTER_VELOCITY_MAP = new InterpolatingDoubleTreeMap();

    static {
      // placeholder values before testing
      // (distance to hub, flywheel voltage)
      RSHOOTER_VELOCITY_MAP.put(1.0, 1.0);
      RSHOOTER_VELOCITY_MAP.put(2.0, 3.0);
      RSHOOTER_VELOCITY_MAP.put(3.0, 5.0);
      RSHOOTER_VELOCITY_MAP.put(4.0, 6.5);
    }
  }

  public class ClimberConstants {}

  public class AgitatorConstants {
    public static final int agitatorMotorID = 0;
    public static final int kickerMotorID = 0;
  }
}
