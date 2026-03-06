package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;

public class TimedShootCommand extends SequentialCommandGroup {
    public TimedShootCommand(ShooterSubsystem shooter, double speed, double revTime, double fireTime) {
        addCommands(
            new RunCommand(() -> shooter.speedUp(speed), shooter)
                .withTimeout(revTime),
            new RunCommand(() -> shooter.fire(speed), shooter)
                .withTimeout(fireTime),
            new RunCommand(() -> shooter.stop(), shooter)
                .withTimeout(0.02)
        );
    }
}