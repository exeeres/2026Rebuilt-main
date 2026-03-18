package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Reverse extends Command {

    private final ShooterSubsystem   shooter;

    private static final double kIdleRPM = 2500; // spins at this when no tag visible

    public Reverse(ShooterSubsystem shooter) {
        this.shooter  = shooter;
    }

    @Override
    public void execute() {
        shooter.reverseFeed();
    }

    @Override
    public boolean isFinished() { return false; } // runs until button released

    @Override
    public void end(boolean interrupted) {
        shooter.stopFeed();
         }
}