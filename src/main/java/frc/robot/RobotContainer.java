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
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Agitator.AgitateAndKick;
import frc.robot.commands.IntakeGoToSetpoint;
import frc.robot.commands.Jostle;
import frc.robot.commands.ResetClimber;
import frc.robot.commands.ResetIntake;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AgitatorSub;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterSubFolder.LFlywheel;
import frc.robot.subsystems.ShooterSubFolder.LFlywheelConfig;
import frc.robot.subsystems.ShooterSubFolder.RFlywheel;
import frc.robot.subsystems.ShooterSubFolder.RFlywheelConfig;
import frc.robot.subsystems.ShooterSubFolder.ShooterSub;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  ShooterSub s_Shooter = new ShooterSub();
  private double MaxSpeed =
      .3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond)
          * 0.4; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final LFlywheel lFlywheel = new LFlywheel(LFlywheelConfig.LFLYWHEEL_CONFIG);
  private final RFlywheel rFlywheel = new RFlywheel(RFlywheelConfig.RFLYWHEEL_CONFIG);

  private final Telemetry logger = new Telemetry(MaxSpeed);
  public final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  private final ShooterSub m_shooter = new ShooterSub();
  private final AgitatorSub m_agitator = new AgitatorSub();
  private final LimelightHelpers m_limelight = new LimelightHelpers();

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
          return 0;
        }
      };

  public RobotContainer() {
    addNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser("Tests");

    autoChooser.addOption("Depot Auto", new PathPlannerAuto("Depot Auto"));
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.getNumber("Climber Pose", m_climber.getClimberPose());

    Dashboard.initialize();

    configureBindings();

    DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withCaptureNt(true));
    DogLog.setPdh(new PowerDistribution());

    SmartDashboard.getNumber("Shooter Limelight TA", LimelightHelpers.getTA("limelight-shooter"));

    // Warmup PathPlanner to avoid Java pauses
    FollowPathCommand.warmupCommand().schedule();
  }

  private void configureBindings() {

    driverController
        .leftTrigger(0.5)
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(
                            -driverController.getLeftY()
                                * MaxSpeed
                                / 0.3) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            -driverController.getLeftX()
                                * MaxSpeed
                                / 0.3) // Drive left with negative X (left)
                        .withRotationalRate(
                            -driverController.getRightX()
                                * MaxAngularRate
                                / 0.4) // Drive counterclockwise with negative X (left)
                ));
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -driverController.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -driverController.getLeftX()
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -driverController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    driverController
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -driverController.getLeftY(), -driverController.getLeftX()))));

    driverController
        .povUp()
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driverController
        .povDown()
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

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

    driverController
        .rightTrigger(0.5)
        .whileTrue(
            Commands.run(() -> lFlywheel.setSetpointVelocity(Shoot.desiredvelocity), lFlywheel)
                .alongWith(
                    Commands.run(
                            () -> rFlywheel.setSetpointVelocity(Shoot.desiredvelocity),
                            rFlywheel) // replace velocity with
                        // LimelightHelpers.getTA("shooter-limelight")
                        .alongWith(
                            new Shoot(
                                lFlywheel,
                                rFlywheel,
                                m_agitator,
                                ShooterConstants.SHOOTER_VELOCITY_MAP.get(
                                    LimelightHelpers.getTA("limelight-shooter"))))
                        .alongWith(drivetrain.applyRequest(() -> brake))
                        .alongWith(new Jostle(m_intake))));

    // driverController.rightTrigger(.5).whileTrue(new
    // FeedForwardCharacterization(flywheel,flywheel::setVoltage, flywheel::getVelocity));

    driverController
        .a()
        .onTrue(new IntakeGoToSetpoint(m_intake, IntakeConstants.intakePivotDownSetpoint));
    driverController
        .x()
        .onTrue(new IntakeGoToSetpoint(m_intake, IntakeConstants.intakePivotUpSetpoint));

    //    driverController
    //        .rightTrigger()
    //        .whileTrue(
    //            new Shoot(
    //                m_shooter,
    //                m_agitator,
    //                ShooterConstants.LSHOOTER_VELOCITY_MAP.get(1.0),
    //                ShooterConstants.RSHOOTER_VELOCITY_MAP.get(1.0)));
    driverController.rightBumper().whileTrue(new RunIntake(m_intake, 6));
    driverController.start().onTrue(new ResetIntake(m_intake));

    driverController.y().whileTrue(new AgitateAndKick(m_agitator, 1, -1));

    operatorController.back().onTrue(new ResetClimber(m_climber));
    operatorController.a().onTrue(new RunClimber(m_climber, ClimberConstants.climberLowSetpoint));
    operatorController.x().onTrue(new RunClimber(m_climber, ClimberConstants.climberClimbSetpoint));
    operatorController
        .y()
        .onTrue(
            new IntakeGoToSetpoint(m_intake, IntakeConstants.intakeClimbSetpoint)
                .andThen(new RunClimber(m_climber, ClimberConstants.climberTopSetpoint)));
    operatorController.leftBumper().whileTrue(new Jostle(m_intake));

    m_agitator.setDefaultCommand(new AgitateAndKick(m_agitator, 1, -1));
    lFlywheel.setDefaultCommand(Commands.run(() -> lFlywheel.setSetpointVelocity(10), lFlywheel));
    rFlywheel.setDefaultCommand(Commands.run(() -> rFlywheel.setSetpointVelocity(10), rFlywheel));

    // Reset the field-centric heading on left bumper press.
    driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

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
        "Intake Up", new IntakeGoToSetpoint(m_intake, IntakeConstants.intakePivotUpSetpoint));
    NamedCommands.registerCommand("Run Intake", new RunIntake(m_intake, 8.5));
    NamedCommands.registerCommand("Agitate And Kick", new AgitateAndKick(m_agitator, 1, -1));

    NamedCommands.registerCommand(
        "Shoot",
        Commands.run(() -> lFlywheel.setSetpointVelocity(Shoot.desiredvelocity), lFlywheel)
            .alongWith(
                Commands.run(
                        () -> rFlywheel.setSetpointVelocity(Shoot.desiredvelocity),
                        rFlywheel) // replace velocity with
                    // LimelightHelpers.getTA("shooter-limelight")
                    .alongWith(
                        new Shoot(
                            lFlywheel,
                            rFlywheel,
                            m_agitator,
                            ShooterConstants.SHOOTER_VELOCITY_MAP.get(
                                LimelightHelpers.getTA("limelight-shooter"))))
                    .alongWith(drivetrain.applyRequest(() -> brake))
                    .alongWith(new Jostle(m_intake)))
            .withTimeout(8));
    NamedCommands.registerCommand(
        "Shoot Preload",
        Commands.run(() -> lFlywheel.setSetpointVelocity(Shoot.desiredvelocity), lFlywheel)
            .alongWith(
                Commands.run(
                        () -> rFlywheel.setSetpointVelocity(Shoot.desiredvelocity),
                        rFlywheel) // replace velocity with
                    // LimelightHelpers.getTA("shooter-limelight")
                    .alongWith(
                        new Shoot(
                            lFlywheel,
                            rFlywheel,
                            m_agitator,
                            ShooterConstants.SHOOTER_VELOCITY_MAP.get(
                                LimelightHelpers.getTA("limelight-shooter"))))
                    .alongWith(drivetrain.applyRequest(() -> brake))
                    .alongWith(new Jostle(m_intake))
                    )
                .withTimeout(5));

    NamedCommands.registerCommand("Climnber to 0",new RunClimber(m_climber, ClimberConstants.climberLowSetpoint));
    NamedCommands.registerCommand("Climber to max",new RunClimber(m_climber, ClimberConstants.climberTopSetpoint));
    NamedCommands.registerCommand("Climber down",new RunClimber(m_climber, ClimberConstants.climberClimbSetpoint));
  }
}
