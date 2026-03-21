package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Feeder extends Command {

    private final ShooterSubsystem   shooter;

    private static final double kIdleRPM = 3000; // spins at this when no tag visible

    public Feeder(ShooterSubsystem shooter) {
        this.shooter  = shooter;
    }

    @Override
    public void execute() {        
        shooter.setRPM(kIdleRPM);
       }

    @Override
    public boolean isFinished() { return false; } // runs until button released

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}