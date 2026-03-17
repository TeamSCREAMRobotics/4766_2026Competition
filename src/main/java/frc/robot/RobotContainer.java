// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.teamscreamrobotics.math.ScreamMath;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Agitator.Agitate;
import frc.robot.commands.Agitator.AgitateAndKick;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.IntakeGoToSetpoint;
import frc.robot.commands.Jostle;
import frc.robot.commands.ManualClimber;
import frc.robot.commands.ResetClimber;
import frc.robot.commands.ResetIntake;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shooter.QuickJostle;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.generated.TunerConstants;
import frc.robot.subsystems.AgitatorSub;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterSubFolder.Flywheel;
import frc.robot.subsystems.ShooterSubFolder.FlywheelConfig;
import frc.robot.subsystems.ShooterSubFolder.ShooterSub;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  ShooterSub s_Shooter = new ShooterSub();
  private double MaxSpeed =
      0.6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setti ng up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //   private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  //   private final SwerveRequest.RobotCentric forwardStraight =
  //       new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final Flywheel m_flywheel = new Flywheel(FlywheelConfig.FLYWHEEL_CONFIG);

  private final Telemetry logger = new Telemetry(MaxSpeed);
  public final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  //   private final ShooterSub m_shooter = new ShooterSub();
  private final AgitatorSub m_agitator = new AgitatorSub();
  //   private final LimelightHelpers m_limelight = new LimelightHelpers();

  Field2d field = new Field2d();

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public static DoubleSupplier desiredFlyWheelVelocity =
      new DoubleSupplier() {

        @Override
        public double getAsDouble() {
          // TODO Auto-generated method stub
          return Dashboard.flywheelVelocity.get();
        }
      };

  public RobotContainer() {
    addNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser("Tests");

    autoChooser.addOption("Depot Auto", new PathPlannerAuto("Depot Auto"));
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

    DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withCaptureNt(true));
    DogLog.setPdh(new PowerDistribution());

    // Warmup PathPlanner to avoid Java pauses
    FollowPathCommand.warmupCommand().schedule();
  }

  private void configureBindings() {

    driverController
        .povUp()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drivetrain
                        .getHelper()
                        .getFacingAngleProfiled(
                            new Translation2d(
                                -driverController.getLeftY(), -driverController.getLeftX()),
                            ScreamMath.calculateAngleToPoint(
                                drivetrain.getState().Pose.getTranslation(),
                                AllianceFlipUtil.get(
                                    FieldConstants.Hub.hubCenter, FieldConstants.Hub.oppHubCenter)),
                            DrivetrainConstants.headingControllerProfiled)));

    driverController
        .povDown()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drivetrain
                        .getHelper()
                        .getFacingAngleProfiled(
                            new Translation2d(
                                -driverController.getLeftY(), -driverController.getLeftX()),
                            new Rotation2d(
                                AllianceFlipUtil.get(Degrees.of(0.0), Degrees.of(180.0))),
                            DrivetrainConstants.headingControllerProfiled)));

    driverController
        .povLeft()
        .whileTrue(
            new DriveToPose(
                drivetrain,
                AllianceFlipUtil.get(
                    new Pose2d(new Translation2d(1.969, 4.158), new Rotation2d(Degrees.of(0.0))),
                    new Pose2d(
                        new Translation2d(14.636, 3.9), new Rotation2d(Degrees.of(180.0))))));
    driverController
        .povRight()
        .whileTrue(
            new DriveToPose(
                drivetrain,
                AllianceFlipUtil.get(
                    new Pose2d(new Translation2d(1.969, 3.3), new Rotation2d(Degrees.of(0.0))),
                    new Pose2d(
                        new Translation2d(14.636, 4.730), new Rotation2d(Degrees.of(180.0))))));
    // driverController
    //     .leftTrigger(0.5)
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 drive
    //                     .withVelocityX(
    //                         -driverController.getLeftY()
    //                             * MaxSpeed
    //                             / 2) // Drive forward with negative Y (forward)
    //                     .withVelocityY(-driverController.getLeftX() * MaxSpeed / 2)));
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -driverController.getLeftY()
                            * MaxSpeed
                            * (driverController.getLeftTriggerAxis() >= 0.5
                                ? 0.5
                                : 1.0)) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -driverController.getLeftX()
                            * MaxSpeed
                            * (driverController.getLeftTriggerAxis() >= 0.5
                                ? 0.5
                                : 1.0)) // Drive left with negative X (left)
                    .withRotationalRate(
                        -driverController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // driverController
    //     .b()
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 point.withModuleDirection(
    //                     new Rotation2d(
    //                         -driverController.getLeftY(), -driverController.getLeftX()))));

    // driverController
    //     .povUp()
    //     .whileTrue(
    //         drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    // driverController
    //     .povDown()
    //     .whileTrue(
    //         drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverController
        .back()
        .and(driverController.y())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController
        .back()
        .and(driverController.x())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController
        .start()
        .and(driverController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController
        .start()
        .and(driverController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    operatorController
        .leftBumper()
        .whileTrue(new RunIntake(m_intake, -6).alongWith(new Agitate(m_agitator, -6)));

    // driverController
    //     .povUp()
    //     .whileTrue(
    //         Commands.run(
    //                 () -> lFlywheel.setSetpointVelocity(ShooterConstants.closeShoot), lFlywheel)
    //             .alongWith(
    //                 Commands.run(
    //                         () -> rFlywheel.setSetpointVelocity(ShooterConstants.closeShoot),
    //                         rFlywheel)
    //                     .alongWith(
    //                         new Shoot40(
    //                             lFlywheel, rFlywheel, m_agitator, ShooterConstants.closeShoot))
    //                     .alongWith(drivetrain.applyRequest(() -> brake))
    //                     .alongWith(new Jostle(m_intake))));

    // driverController
    //     .povLeft()
    //     .whileTrue(
    //         Commands.run(() -> lFlywheel.setSetpointVelocity(ShooterConstants.midShoot),
    // lFlywheel)
    //             .alongWith(
    //                 Commands.run(
    //                         () -> rFlywheel.setSetpointVelocity(ShooterConstants.midShoot),
    //                         rFlywheel)
    //                     .alongWith(
    //                         new Shoot50(
    //                             lFlywheel, rFlywheel, m_agitator, ShooterConstants.midShoot))
    //                     .alongWith(drivetrain.applyRequest(() -> brake))
    //                     .alongWith(new Jostle(m_intake))));

    // driverController
    //     .povDown()
    //     .whileTrue(
    //         Commands.run(() -> lFlywheel.setSetpointVelocity(ShooterConstants.farShoot),
    // lFlywheel)
    //             .alongWith(
    //                 Commands.run(
    //                         () -> rFlywheel.setSetpointVelocity(ShooterConstants.farShoot),
    //                         rFlywheel)
    //                     .alongWith(
    //                         new Shoot60(
    //                             lFlywheel, rFlywheel, m_agitator, ShooterConstants.farShoot))
    //                     .alongWith(drivetrain.applyRequest(() -> brake))
    //                     .alongWith(new Jostle(m_intake))));

    // driverController.rightTrigger(.5).whileTrue(new
    // FeedForwardCharacterization(flywheel,flywheel::setVoltage, flywheel::getVelocity));
    driverController
        .rightTrigger(.5)
        .whileTrue(
            Commands.run(
                    () ->
                        m_flywheel.setSetpointVelocity(
                            ShooterConstants.SHOOTER_VELOCITY_MAP.get(getShooterDistance())),
                    m_flywheel)
                .alongWith(new Shoot(m_flywheel, m_agitator, getDesiredShooterVelocity))
                .alongWith(drivetrain.applyRequest(() -> brake))
                .alongWith(new Jostle(m_intake)));

    driverController
        .a()
        .onTrue(new IntakeGoToSetpoint(m_intake, IntakeConstants.intakePivotDownSetpoint));
    driverController
        .x()
        .onTrue(new IntakeGoToSetpoint(m_intake, IntakeConstants.intakePivotUpSetpoint));

    driverController
        .rightBumper()
        .whileTrue(
            new RunIntake(m_intake, 7)
                .alongWith(
                    new Agitate(m_agitator, 2)
                        .alongWith(
                            drivetrain.applyRequest(
                                () ->
                                    drive
                                        .withVelocityX(
                                            -driverController.getLeftY()
                                                * MaxSpeed
                                                * 0.5) // Drive forward with negative Y (forward)
                                        .withVelocityY(
                                            -driverController.getLeftX()
                                                * MaxSpeed
                                                * 0.5) // Drive left with negative X (left)
                                        .withRotationalRate(
                                            -driverController.getRightX() * MaxAngularRate)))));
    driverController.start().onTrue(new ResetIntake(m_intake));

    operatorController.back().onTrue(new ResetClimber(m_climber));
    operatorController.a().onTrue(new RunClimber(m_climber, ClimberConstants.climberLowSetpoint));
    operatorController.x().onTrue(new RunClimber(m_climber, ClimberConstants.climberClimbSetpoint));
    operatorController
        .y()
        .onTrue(
            new IntakeGoToSetpoint(m_intake, IntakeConstants.intakeClimbSetpoint)
                .andThen(new RunClimber(m_climber, ClimberConstants.climberTopSetpoint)));
    operatorController.leftTrigger().whileTrue(new QuickJostle(m_intake));
    operatorController.povDown().whileTrue(new ManualClimber(m_climber, -1));
    operatorController.povUp().whileTrue(new ManualClimber(m_climber, 1));
    operatorController
        .rightTrigger(0.5)
        .whileTrue(
            Commands.run(
                    () -> m_flywheel.setSetpointVelocity(Dashboard.flywheelVelocity.get()),
                    m_flywheel)
                .alongWith(new Shoot(m_flywheel, m_agitator, getDesiredShooterVelocity))
                .alongWith(new Jostle(m_intake)));

    m_agitator.setDefaultCommand(
        Commands.run(() -> m_agitator.RunAgitatorAndKicker(-2, 0), m_agitator));
    m_flywheel.setDefaultCommand(
        Commands.run(
            () -> m_flywheel.setSetpointVelocity(getDesiredShooterVelocity.getAsDouble() * 0.25),
            m_flywheel));

    // Reset the field-centric heading on left bumper press.
    driverController
        .leftBumper()
        .and(driverController.start())
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  public void addNamedCommands() {
    NamedCommands.registerCommand(
        "Intake Down", new IntakeGoToSetpoint(m_intake, IntakeConstants.intakePivotDownSetpoint));
    NamedCommands.registerCommand(
        "Intake Up",
        new IntakeGoToSetpoint(m_intake, IntakeConstants.intakePivotUpSetpoint).withTimeout(1));
    NamedCommands.registerCommand("Run Intake", new RunIntake(m_intake, 8.5).withTimeout(2));
    NamedCommands.registerCommand("Agitate And Kick", new AgitateAndKick(m_agitator, 1, -1));

    NamedCommands.registerCommand(
        "Shoot",
        Commands.run(
                () -> m_flywheel.setSetpointVelocity(Shoot.desiredvelocity.getAsDouble()),
                m_flywheel)
            .alongWith(new Shoot(m_flywheel, m_agitator, getDesiredShooterVelocity))
            .alongWith(drivetrain.applyRequest(() -> brake))
            .alongWith(new Jostle(m_intake))
            .withTimeout(6.5));
    NamedCommands.registerCommand(
        "Shoot Preload",
        Commands.run(
                () -> m_flywheel.setSetpointVelocity(Shoot.desiredvelocity.getAsDouble()),
                m_flywheel)
            .alongWith(new Shoot(m_flywheel, m_agitator, getDesiredShooterVelocity))
            .alongWith(drivetrain.applyRequest(() -> brake))
            .alongWith(new Jostle(m_intake))
            .withTimeout(5));

    NamedCommands.registerCommand(
        "Climnber to 0", new RunClimber(m_climber, ClimberConstants.climberLowSetpoint));
    NamedCommands.registerCommand(
        "Climber to max",
        (Commands.run(() -> m_climber.climberGoToSetpoint(ClimberConstants.climberTopSetpoint)))
            .withTimeout(2));
    NamedCommands.registerCommand(
        "Climber down", new RunClimber(m_climber, ClimberConstants.climberClimbSetpoint));
  }

  public double getShooterDistance() {
    /*  double botX = drivetrain.getPose().getX();
    double botY = drivetrain.getPose().getY();
    double targetX = Hub.topCenterPoint.getX();
    double targetY = Hub.topCenterPoint.getY(); */

    // return Math.sqrt(Math.pow(targetX - botX, 2) + Math.pow(targetY - botY, 2));
    return drivetrain
        .getPose()
        .getTranslation()
        .getDistance(
            AllianceFlipUtil.get(
                Hub.topCenterPoint.toTranslation2d(), Hub.oppTopCenterPoint.toTranslation2d()));
  }

  public DoubleSupplier getDesiredShooterVelocity =
      new DoubleSupplier() {

        @Override
        public double getAsDouble() {
          return ShooterConstants.SHOOTER_VELOCITY_MAP.get(getShooterDistance());
        }
      };

  public void Periodic() {
    field.setRobotPose(drivetrain.getPose());
    SmartDashboard.putData(field);
    SmartDashboard.getNumber("Climber Pose", m_climber.getClimberPose());
    SmartDashboard.putNumber("Flywheel RPS", m_flywheel.getVelocity());
    SmartDashboard.putNumber("Flywheel RPM", m_flywheel.getVelocity() * 60);
    SmartDashboard.putNumber("Calculated Distance", this.getShooterDistance());
    SmartDashboard.putNumber(
        "Treemap Velocity", ShooterConstants.SHOOTER_VELOCITY_MAP.get(this.getShooterDistance()));
    Dashboard.initialize();
  }
}

//    return drivetrain.getPose().getTranslation().getDistance(
        // AllianceFlipUtil.get(
        // Hub.hubCenter,
        // Hub.oppHubCenter));
