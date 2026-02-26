// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.teamscreamrobotics.zones.RectangularPoseArea;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class Constants {

  public class DriveTrainConstants {}

  public class VisionConstants {}

  public class IntakeConstants {
    public static final int intakeMotorID = 14;
    public static final int intakePivotID = 12;
    public static final double kP = 1.7;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kV = 0;
    public static final double kG = 0;
    public static final double intakeMagicAcceleration = 26;
    public static final double intakeMagicVelocity = 30;
    public static final double intakePivotDownSetpoint = 7.1;
    public static final double intakePivotUpSetpoint = 0.33;
    public static final double intakeAgitateHighSetpoint = 3.5;
    public static final double intakeAgiteLowSetpoint = 3.0;
    public static final double intakePivotForwardSoftLimit = 7.33;
    public static final double intakePivotReversSoftLimit = 0.005;
  }

  public class ShooterConstants {
    public static final int LshooterMoterID = 17; // testing on testbed
    public static final int RshooterMotorID = 16;
    public static final int shooterCANID = 0;

    public static final int pivotMotorID = 0;
    public static final int pivotCanID = 0;

    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kP = .73;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kMagicAcceleration = 60;
    public static final double kMagicCruiseVelocity = 80;

    public static final double defaultVelocity = 50.0;

    //  public static final InterpolatingDoubleTreeMap HOOD_MAP = new InterpolatingDoubleTreeMap();
    //
    //  static {
    //    // placeholder values before testing
    //    // (distance to hub, hood angle)
    //    HOOD_MAP.put(1.0, 10.0);
    //    HOOD_MAP.put(2.0, 20.0);
    //    HOOD_MAP.put(3.0, 30.0);
    //    HOOD_MAP.put(4.0, 40.0);
    //  }

    public static final InterpolatingDoubleTreeMap LSHOOTER_VELOCITY_MAP =
        new InterpolatingDoubleTreeMap();

    static {
      // placeholder values before testing
      // (distance to hub, flywheel voltage)
      LSHOOTER_VELOCITY_MAP.put(1.0, 8.0);
      LSHOOTER_VELOCITY_MAP.put(2.0, 3.0);
      LSHOOTER_VELOCITY_MAP.put(3.0, 5.0);
      LSHOOTER_VELOCITY_MAP.put(4.0, 6.5);
    }

    public static final InterpolatingDoubleTreeMap RSHOOTER_VELOCITY_MAP =
        new InterpolatingDoubleTreeMap();

    static {
      // placeholder values before testing
      // (distance to hub, flywheel voltage)
      RSHOOTER_VELOCITY_MAP.put(1.0, 8.0);
      RSHOOTER_VELOCITY_MAP.put(2.0, 3.0);
      RSHOOTER_VELOCITY_MAP.put(3.0, 5.0);
      RSHOOTER_VELOCITY_MAP.put(4.0, 6.5);
    }
  }

  public class ClimberConstants {
    public static final int climbermotorID = 20;
    public static final double kP = 12.9;
    public static final double kI = 0;
    public static final double kD = 0.5;
    public static final double kV = 0;
    public static final double kG = 0;
    public static final double climberTopSetpoint = 8.25;
    public static final double climberLowSetpoint = 0.008;
    public static final double climberClimbSetpoint = 6;
    public static final double climberReverseThreshold = 0.005;
    public static final double climberForwardThreshold = 8.258;

    public static final double climberMagicAccereation = 1;
    public static final double climberMagicCruiseVelocity = 1;
  }

  public class AgitatorConstants {
    public static final int agitatorMotorID = 13;
    public static final int kickerMotorID = 15;
  }

  public class FieldConstants {
    public static final Translation2d fieldDimesions = new Translation2d(null, null);
    public static final RectangularPoseArea fieldArea =
        new RectangularPoseArea(Translation2d.kZero, fieldDimesions);
    public static final Pose2d blueHubAlign =
        new Pose2d(0, fieldDimesions.getY(), Rotation2d.fromDegrees(0));
    public static final Pose2d redHubAlign =
        new Pose2d(fieldDimesions.getX() - 0, fieldDimesions.getY() * 0, null);

    private static Pair<Integer, Pose2d> getTagPair(int id) {
      return Pair.of(
          id,
          AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)
              .getTagPose(id)
              .get()
              .toPose2d());
    }
  }
}
