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

        Commands.runOnce(() -> shooter.speedUp(ShooterConstants.autoShooterSpeed, 1.37), shooter),
        new WaitCommand(1.37),

        Commands.runOnce(() -> shooter.fire(ShooterConstants.autoShooterSpeed, 2), shooter),
        new WaitCommand(4.5),

        Commands.run(() -> intake.timedRun(1.5), intake),
        new WaitCommand(2.87),

        Commands.runOnce(() -> shooter.speedUp(ShooterConstants.autoShooterSpeed, 1), shooter),
        new WaitCommand(1),

        Commands.runOnce(() -> shooter.fire(ShooterConstants.autoShooterSpeed, 2), shooter)

        );  
    }


}
