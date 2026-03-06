package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RedRightMultiAuto extends SequentialCommandGroup{
    public RedRightMultiAuto(CommandSwerveDrivetrain swerve, ShooterSubsystem shooter, WristSubsystem wrist, IntakeSubsystem intake){
    addCommands(
        new PathPlannerAuto("RedRightMultiShoot"),

        new TimedShootCommand(shooter, ShooterConstants.autoShooterSpeed, 1.37, 2.0),
        new WaitCommand(1.37),

        new WaitCommand(4.5),

        new TimedIntakeCommand(intake, 1.5),
        new WaitCommand(2.87),

        new TimedShootCommand(shooter, ShooterConstants.autoShooterSpeed, 1.0, 2.0)


        );  
    }


}
