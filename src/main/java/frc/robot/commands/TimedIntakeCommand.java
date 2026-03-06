package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class TimedIntakeCommand extends SequentialCommandGroup {
    public TimedIntakeCommand(IntakeSubsystem intakeSubsystem, double runTime) {
        addCommands(
            new RunCommand(() -> intakeSubsystem.run(), intakeSubsystem)
                .withTimeout(runTime),
            new RunCommand(() -> intakeSubsystem.stop(), intakeSubsystem)
                .withTimeout(0.02)
        );
    }
}