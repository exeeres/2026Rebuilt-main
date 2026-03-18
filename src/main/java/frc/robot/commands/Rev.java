package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Rev extends Command {

    private final ShooterSubsystem   shooter;
    private final LimelightSubsystem limelight;

    private static final double kIdleRPM = 2500; // spins at this when no tag visible

    public Rev(ShooterSubsystem shooter, LimelightSubsystem limelight) {
        this.shooter  = shooter;
        this.limelight = limelight;
        
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double distance = limelight.getDistanceToTarget();
        
        if (distance > 0) {
            shooter.setRPMFromDistance(distance);
        } else {
            shooter.setRPM(kIdleRPM);
        }

       }

    @Override
    public boolean isFinished() { return false; } // runs until button released

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}