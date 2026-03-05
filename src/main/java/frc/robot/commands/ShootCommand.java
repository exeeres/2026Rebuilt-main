package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootCommand extends Command {
  
  private final ShooterSubsystem m_shootersubsystem;
  private final double shootTime;

  public ShootCommand(ShooterSubsystem subsystem, double shoottime) {
    m_shootersubsystem = subsystem;
    shootTime = shoottime;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}
  @Override
  public void execute(){
    m_shootersubsystem.fire(ShooterConstants.shootSpeed, shootTime);
  }
  @Override
  public boolean isFinished() {
    m_shootersubsystem.stop();
    return true;
  }
}