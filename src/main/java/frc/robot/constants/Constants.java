// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

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
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  public class VisionConstants {
    public static final int resolutionWidth = 1280;
    public static final int resolutionHeight = 960;

    public static final double xyStdBaseline = 0.65;
    public static final double thetaStdBaseline = 0.04;

    public static final double xyMt2StdFactor = 0.35;
  }

  public class IntakeConstants {
    public static final int intakeMotorID = 14;
    public static final int intakePivotID = 12;
    public static final double kP = 3.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kV = 0;
    public static final double kG = 0;
    public static final double kS = 1;
    public static final double intakeMagicAcceleration = 52;
    public static final double intakeMagicVelocity = 60;

    public static final double intakePivotDownSetpoint = 11.26;
    public static final double intakePivotUpSetpoint = 0.15;

    public static final double intakeClimbSetpoint = 0.1;

    public static final double intakeAgitateHighSetpoint = 6.0;
    public static final double intakeAgitateLowSetpoint = 9.88;

    public static final double intakePivotForwardSoftLimit = 11.3;
    public static final double intakePivotReverseSoftLimit = -0.3;
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

    public static final double closeShoot = 40.0;
    public static final double midShoot = 50.0;
    public static final double farShoot = 60.0;

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

    public static final InterpolatingDoubleTreeMap SHOOTER_VELOCITY_MAP =
        new InterpolatingDoubleTreeMap();

    static {
      SHOOTER_VELOCITY_MAP.put(0.0, 37.0);
      SHOOTER_VELOCITY_MAP.put(1.172617836430566, 37.0);
      SHOOTER_VELOCITY_MAP.put(1.346530562720079, 39.0);
      SHOOTER_VELOCITY_MAP.put(1.478706275947643, 40.0);
      SHOOTER_VELOCITY_MAP.put(2.2684219219170934, 42.0);
      SHOOTER_VELOCITY_MAP.put(2.055824253850473, 44.25);
      SHOOTER_VELOCITY_MAP.put(2.2183737723024715, 45.0);
      SHOOTER_VELOCITY_MAP.put(2.3117374055228126, 46.5);
      SHOOTER_VELOCITY_MAP.put(2.380634439956445, 48.5);
      SHOOTER_VELOCITY_MAP.put(2.7278039312097353, 51.0);
      SHOOTER_VELOCITY_MAP.put(2.683809483762821, 51.5);
      SHOOTER_VELOCITY_MAP.put(3.1532988426000435, 51.875);
      SHOOTER_VELOCITY_MAP.put(3.4170126639197895, 55.0);
      SHOOTER_VELOCITY_MAP.put(3.675951047182713, 57.0);
      SHOOTER_VELOCITY_MAP.put(3.856427421283271, 58.75);
      SHOOTER_VELOCITY_MAP.put(4.131069897490271, 59.5);
    }

    public static final InterpolatingDoubleTreeMap FERRY_VELOCITY_MAP =
        new InterpolatingDoubleTreeMap();

    static {
      FERRY_VELOCITY_MAP.put(0.0, 37.0 * 1.5);
      FERRY_VELOCITY_MAP.put(1.172617836430566, 37.0* 1.5);
      FERRY_VELOCITY_MAP.put(1.346530562720079, 39.0* 1.5);
      FERRY_VELOCITY_MAP.put(1.478706275947643, 40.0* 1.5);
      FERRY_VELOCITY_MAP.put(2.2684219219170934, 42.0* 1.5);
      FERRY_VELOCITY_MAP.put(2.055824253850473, 44.25* 1.5);
      FERRY_VELOCITY_MAP.put(2.2183737723024715, 45.0* 1.5);
      FERRY_VELOCITY_MAP.put(2.3117374055228126, 46.5* 1.5);
      FERRY_VELOCITY_MAP.put(2.380634439956445, 48.5* 1.5);
      FERRY_VELOCITY_MAP.put(2.7278039312097353, 51.0* 1.5);
      FERRY_VELOCITY_MAP.put(2.683809483762821, 51.5* 1.5);
      FERRY_VELOCITY_MAP.put(3.1532988426000435, 51.875* 1.5);
      FERRY_VELOCITY_MAP.put(3.4170126639197895, 55.0* 1.5);
      FERRY_VELOCITY_MAP.put(3.675951047182713, 57.0* 1.5);
      FERRY_VELOCITY_MAP.put(3.856427421283271, 58.75* 1.5);
      FERRY_VELOCITY_MAP.put(4.131069897490271, 59.5* 1.5);
    }

    // public static final InterpolatingDoubleTreeMap RSHOOTER_VELOCITY_MAP =
    //     new InterpolatingDoubleTreeMap();

    // static {
    //   // placeholder values before testing
    //   // (distance to hub, flywheel voltage)
    //   RSHOOTER_VELOCITY_MAP.put(1.0, 8.0);
    //   RSHOOTER_VELOCITY_MAP.put(2.0, 3.0);\[]

    //   RSHOOTER_VELOCITY_MAP.put(3.0, 5.0);
    //   RSHOOTER_VELOCITY_MAP.put(4.0, 6.5);
    // }
  }

  public class ClimberConstants {
    public static final int climbermotorID = 20;
    public static final double kP = 70;
    public static final double kI = 0;
    public static final double kD = 0.7;
    public static final double kV = 0;
    public static final double kG = 0;
    public static final double kS = 0.396;
    public static final double climberTopSetpoint = 8.7;
    public static final double climberLowSetpoint = 0.0;
    public static final double climberClimbSetpoint = 4;
    public static final double climberReverseThreshold = -0.005;
    public static final double climberForwardThreshold = 8.75;

    public static final double climberMagicAccereation = 6;
    public static final double climberMagicCruiseVelocity = 2;
  }

  public class IndexerConstants {
    public static final int agitatorMotorID = 13;
    public static final int kickerMotorID = 15;
  }

  public class FieldConstants {
    public static final Translation2d fieldDimesions = new Translation2d(16.54, 8);
    public static final RectangularPoseArea fieldArea =
        new RectangularPoseArea(Translation2d.kZero, fieldDimesions);
    public static final Pose2d blueHubAlign =
        new Pose2d(0, fieldDimesions.getY(), Rotation2d.fromDegrees(0));
    public static final Pose2d redHubAlign =
        new Pose2d(fieldDimesions.getX() - 0, fieldDimesions.getY() * 0, null);

    @SuppressWarnings("unused")
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
