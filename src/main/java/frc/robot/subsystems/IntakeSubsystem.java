package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase{
    private SparkFlex intake;

    public IntakeSubsystem() {
        intake = new SparkFlex(IntakeConstants.intakeID, MotorType.kBrushless);
    }

    public void run (){
        intake.set(0.8);
    }

    public void stop(){
        intake.set(0);
    }

}
