package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class SetWristCommand extends Command {
    public WristSubsystem m_wrist;
    public double point;
    public boolean done;
    
    public SetWristCommand(WristSubsystem subsystem, double setPoint) {
       m_wrist = subsystem;
       point = setPoint;

       addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
        m_wrist.setPosition(point);

        done = true; //hiiiiiiiiiiiii
    }

    @Override
    public boolean isFinished() {
        return done;
    }
    
}
