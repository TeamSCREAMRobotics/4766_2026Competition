// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.teamscreamrobotics.util.GeomUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> targetPose;

  private final ProfiledPIDController driveController =
      DrivetrainConstants.driveAlignmentController;
  private final PIDController headingController = DrivetrainConstants.headingController;

  private double driveErrorAbs = 1.0;
  private Translation2d lastSetpointTranslation;

  private Optional<DoubleSupplier> yOverride = Optional.empty();
  private Optional<Supplier<Translation2d>> translationOverride = Optional.empty();

  /** Creates a new DriveToPose. */
  public DriveToPose(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    addRequirements(drivetrain);
    setName("DriveToPose");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public DriveToPose(
      CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetPose, DoubleSupplier yOverride) {
    this(drivetrain, targetPose);
    this.yOverride = Optional.of(yOverride);
  }

  public DriveToPose(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Pose2d> targetPose,
      Supplier<Translation2d> translationOverride) {
    this(drivetrain, targetPose);
    this.translationOverride = Optional.of(translationOverride);
  }

  public DriveToPose(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    this(drivetrain, () -> targetPose);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = drivetrain.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(targetPose.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(drivetrain.getFieldVelocity().dx, drivetrain.getFieldVelocity().dy)
                .rotateBy(
                    targetPose
                        .get()
                        .getTranslation()
                        .minus(drivetrain.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    headingController.reset();
    lastSetpointTranslation = currentPose.getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    Pose2d targetPose = this.targetPose.get();

    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler = MathUtil.clamp((currentDistance - 0.2) / (0.8 - 0.2), 0.0, 1.0);
    driveErrorAbs = currentDistance;

    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    double driveVelocity =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    double headingVelocity =
        headingController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    Translation2d velocity;

    if (translationOverride.isPresent() && translationOverride.get().get().getNorm() > 0.5) {
      velocity = translationOverride.get().get();
    } else {
      velocity =
          new Pose2d(
                  new Translation2d(),
                  currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
              .transformBy(GeomUtil.translationToTransform(driveVelocity, 0.0))
              .getTranslation();
    }

    drivetrain.setControl(
        drivetrain
            .getHelper()
            .getApplyFieldSpeeds(
                new ChassisSpeeds(
                    velocity.getX(),
                    yOverride.isPresent() ? yOverride.get().getAsDouble() : velocity.getY(),
                    headingVelocity)));

    DogLog.log("DriveToPose/MeasuredDistance", currentDistance);
    DogLog.log("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    DogLog.log("DriveToPose/MeasuredHeading", currentPose.getRotation().getDegrees());
    DogLog.log("DriveToPose/SetpointHeading", targetPose.getRotation().getDegrees());
    DogLog.log(
        "DriveToPose/Setpoint",
        new Pose2d(lastSetpointTranslation, new Rotation2d(headingController.getSetpoint())));
    DogLog.log("DriveToPose/TargetPose", targetPose);
  }

  public boolean atGoal() {
    return driveController.atGoal() && headingController.atSetpoint();
  }
}
