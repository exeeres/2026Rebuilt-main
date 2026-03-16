package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RevandFeedCommand extends Command {

    private final ShooterSubsystem   shooter;
    private final LimelightSubsystem limelight;

    private static final double kIdleRPM = 2500; // spins at this when no tag visible

    public RevandFeedCommand(ShooterSubsystem shooter, LimelightSubsystem limelight) {
        this.shooter  = shooter;
        this.limelight = limelight;
        
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (limelight.hasTarget()) {
            // Plug limelight distance straight into RPM equation
            shooter.setRPMFromDistance(limelight.getDistanceToTarget());
        } else {
            // No tag — keep wheels warm at idle
            shooter.setRPM(kIdleRPM);
        }

        if (shooter.atTargetRPM()) {
            shooter.runFeed();
        } else {
            shooter.stopFeed();
        }
    }

    @Override
    public boolean isFinished() { return false; } // runs until button released

    @Override
    public void end(boolean interrupted) {
        shooter.stopFeed();
        shooter.stop();
    }
}