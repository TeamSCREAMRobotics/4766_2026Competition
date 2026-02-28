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
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  public class DriveTrainConstants {}

  public class VisionConstants {
      public static final int resolutionWidth = 1280;
  public static final int resolutionHeight = 960;

  public static final double xyStdBaseline = 0.9;
  public static final double thetaStdBaseline = 0.06;

  public static final double xyMt2StdFactor = 0.3;
  }

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
    public static final double intakeClimbSetpoint = 0;
    public static final double intakeAgitateHighSetpoint = 3.0;
    public static final double intakeAgitateLowSetpoint = 4.5;
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

    public static final InterpolatingDoubleTreeMap SHOOTER_VELOCITY_MAP =
        new InterpolatingDoubleTreeMap();

    static {
      // placeholder values before testing
      // (distance to hub, flywheel voltage)
      SHOOTER_VELOCITY_MAP.put(0.8644093871116638, 50.0);
      SHOOTER_VELOCITY_MAP.put(0.215736523270607, 60.0);
      SHOOTER_VELOCITY_MAP.put(0.46821823716163635, 58.0);
      SHOOTER_VELOCITY_MAP.put(1.643693447113037, 43.0);
      SHOOTER_VELOCITY_MAP.put(0.6914072632789612, 53.0);
      SHOOTER_VELOCITY_MAP.put(1.7289714813232422, 44.0);
      SHOOTER_VELOCITY_MAP.put(1.1395137310028076, 48.0);
      SHOOTER_VELOCITY_MAP.put(0.5004304647445679, 55.5);
      SHOOTER_VELOCITY_MAP.put(0.4286656975746155, 59.0);
      SHOOTER_VELOCITY_MAP.put(0.5435771346092224, 57.0);
      SHOOTER_VELOCITY_MAP.put(0.25983139872550964, 63.0); // could be wrong
      SHOOTER_VELOCITY_MAP.put(0.27644532918930054, 63.5);
      SHOOTER_VELOCITY_MAP.put(0.5533760786056519, 50.0);
      SHOOTER_VELOCITY_MAP.put(0.18435527384281158, 68.0);
    }

    // public static final InterpolatingDoubleTreeMap RSHOOTER_VELOCITY_MAP =
    //     new InterpolatingDoubleTreeMap();

    // static {
    //   // placeholder values before testing
    //   // (distance to hub, flywheel voltage)
    //   RSHOOTER_VELOCITY_MAP.put(1.0, 8.0);
    //   RSHOOTER_VELOCITY_MAP.put(2.0, 3.0);
    //   RSHOOTER_VELOCITY_MAP.put(3.0, 5.0);
    //   RSHOOTER_VELOCITY_MAP.put(4.0, 6.5);
    // }
  }

  public class ClimberConstants {
    public static final int climbermotorID = 20;
    public static final double kP = 50;
    public static final double kI = 0;
    public static final double kD = 0.7;
    public static final double kV = 0;
    public static final double kG = 0;
    public static final double kS = 0.396;
    public static final double climberTopSetpoint = 8.7;
    public static final double climberLowSetpoint = 0.0;
    public static final double climberClimbSetpoint = 5;
    public static final double climberReverseThreshold = 0.005;
    public static final double climberForwardThreshold = 8.75;

    public static final double climberMagicAccereation = 2;
    public static final double climberMagicCruiseVelocity = 2;
  }

  public class AgitatorConstants {
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
