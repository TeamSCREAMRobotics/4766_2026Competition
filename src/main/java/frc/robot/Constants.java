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

/** Add your docs here. */
public class Constants {

  public class DriveTrainConstants {}

  public class VisionConstants {}

  public class IntakeConstants {
    public static final int intakeMotorID = 6;
    public static final int intakePivotID = 7;
    public static final int kP = 0;
    public static final int kI = 0;
    public static final int kD = 0;
    public static final int kV = 0;
    public static final int kG = 0;
    public static final double intakeMagicAcceleration = 0;
    public static final double intakeMagicVelocity = 0;
  }

  public class ShooterConstants {}

  public class ClimberConstants {
    public static final int climbermotorID = 20;
    public static final int kP = 0;
    public static final int kI = 0;
    public static final int kD = 0;
    public static final int kV = 0;
    public static final int kG = 0;
    public static final int climberTopSetpoint = 0;
    public static final int climberLowSetpoint = 0;
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

  public class AgitatorConstants {}
}
