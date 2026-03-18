// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
//import frc.robot.commands.autos.BlueLeftDouble;
import frc.robot.commands.lime.AlignToTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.Feed;
import frc.robot.commands.Rev;
import frc.robot.commands.Reverse;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    private static final double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.OperatorConstants.DEADBAND)
            .withRotationalDeadband(MaxAngularRate * Constants.OperatorConstants.DEADBAND) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController kdriverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
    private final static CommandXboxController koperatorController = new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);
        // establish subsystem auto after robot is built (likely 5 weeks from 2/19/2026)
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LimelightSubsystem limelight = new LimelightSubsystem(drivetrain);
    public final ShooterSubsystem shootersubsystem = new ShooterSubsystem();
    public final WristSubsystem wrist = new WristSubsystem(koperatorController::getLeftTriggerAxis);
    public final IntakeSubsystem intake = new IntakeSubsystem(koperatorController::getRightTriggerAxis);
    
        public RobotContainer() {
            autoChooser = new SendableChooser<Command>();
            //autoChooser.addOption("BlueLeft", new BlueLeftDouble(drivetrain, shootersubsystem, wrist, intake));
            SmartDashboard.putData("Auto Chooser", autoChooser);
            configureBindings();
        }
        
        public static double getRightTriggerValue(){
            return koperatorController.getRightTriggerAxis();
        }
        public static double getLeftTriggerValue(){
            return koperatorController.getLeftTriggerAxis();
        }
        public static double getABoolean(){
            return koperatorController.a().getAsBoolean() ? 1 : 0;
        }
        

    private void configureBindings() {

        kdriverController.x().whileTrue(new AlignToTag(drivetrain, limelight, () -> -kdriverController.getLeftY(),  // forward/back
            () -> -kdriverController.getLeftX()));
        koperatorController.rightBumper().whileTrue(new Rev(shootersubsystem, limelight));
        koperatorController.a().whileTrue(Commands.startEnd(
            () -> shootersubsystem.enableTuningMode(),
            () -> {shootersubsystem.disableTuningMode();
            shootersubsystem.stop();}));
        koperatorController.leftBumper().whileTrue(new Feed(shootersubsystem));
        koperatorController.b().whileTrue(new Reverse(shootersubsystem));


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-kdriverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-kdriverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-kdriverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        kdriverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        kdriverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-kdriverController.getLeftY(), -kdriverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        kdriverController.back().and(kdriverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        kdriverController.back().and(kdriverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        kdriverController.start().and(kdriverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        kdriverController.start().and(kdriverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        kdriverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}