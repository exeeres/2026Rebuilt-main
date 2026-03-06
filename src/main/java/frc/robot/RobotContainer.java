// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.RedRightMultiAuto;
import frc.robot.commands.RevCommand;
import frc.robot.commands.SetWristCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.lime.AlignToTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.OperatorConstants.DEADBAND)
            .withRotationalDeadband(MaxAngularRate * Constants.OperatorConstants.DEADBAND) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SendableChooser<Command> autoChooser;

    private static final double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController kdriverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
    private final static CommandXboxController koperatorController = new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LimelightSubsystem limelight = new LimelightSubsystem(drivetrain);
    public final ShooterSubsystem shootersubsystem = new ShooterSubsystem();
    public final WristSubsystem wrist = new WristSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    
    public RobotContainer() {        
        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("Red Right Multi", new RedRightMultiAuto(drivetrain, shootersubsystem, wrist, intake));
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }
        
    public static double getRightTriggerValue(){
        return koperatorController.getRightTriggerAxis();
    }
    public static double getLeftTriggerValue(){
        return koperatorController.getLeftTriggerAxis();
    }

    private void configureBindings() {

        koperatorController.a().onTrue(new ShootCommand(shootersubsystem, ShooterConstants.shootTime));
        koperatorController.b().onTrue(new RevCommand(shootersubsystem, ShooterConstants.revTime));
        koperatorController.rightTrigger(0.1).whileTrue(new RunCommand(() -> intake.run(), intake))
            .onFalse(new RunCommand(() -> intake.stop(), intake)
                .withTimeout(0.02));

        /*wrist.setDefaultCommand(new RunCommand(() -> {
            double leftY = koperatorController.getLeftY();
            if (leftY > 0.1) {
                wrist.rotateOut();
            } else if (leftY < -0.1) {
                wrist.rotateIn();
            }
        }, wrist));*/

        //koperatorController.y().onTrue(new SetWristCommand(wrist, -2));
        koperatorController.x().whileTrue(new AlignToTag(drivetrain, limelight, () -> -kdriverController.getLeftY(),  // forward/back
                () -> -kdriverController.getLeftX()));
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
    return null;
        //return autoChooser.getSelected();
  }
}