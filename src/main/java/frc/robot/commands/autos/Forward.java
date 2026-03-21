package frc.robot.commands.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.WristSubsystem;

public class Forward extends SequentialCommandGroup{
    public Forward(CommandSwerveDrivetrain drivetrain, WristSubsystem wrist) {
        addCommands(
            new PathPlannerAuto("Forward"),

            Commands.runOnce(() -> wrist.setPosition(WristConstants.wristOutPostion), wrist)
        );
    }
}
