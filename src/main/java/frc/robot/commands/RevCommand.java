package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevCommand extends Command{
      
  private final ShooterSubsystem m_shootersubsystem;
  private final double revTime;

  public RevCommand(ShooterSubsystem subsystem, double revtime) {
    m_shootersubsystem = subsystem;
    revTime = revtime;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {  }
  @Override
  public void execute(){
    m_shootersubsystem.speedUp(ShooterConstants.shootSpeed, revTime);
  }

  @Override
  public boolean isFinished() {
    m_shootersubsystem.stop();
    return true;
  }
}
