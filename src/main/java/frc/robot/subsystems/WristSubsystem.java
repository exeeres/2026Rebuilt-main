package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase{
    
    private SparkFlex wristMotor;
    /*
     * Clockwise = Down 
     * Counter Clockwise = Up
     */
    
     private PIDController pidController;

    private RelativeEncoder wristEncoder;

    private double position;

    private double maxLimit;
    private double minLimit;

    public WristSubsystem(){
        wristMotor = new SparkFlex(WristConstants.wristID, MotorType.kBrushless);

        wristEncoder = wristMotor.getEncoder();

        pidController = new PIDController(0.4, 0.0, 0.0);
        pidController.setTolerance(0.1);

        maxLimit = 15;
        minLimit = -15;
    }

    public void rotateIn(){
        position = position + 0.2;
        if(position > maxLimit){
            position = maxLimit;
        }
        if(position < minLimit){
            position = minLimit;
        }
    }
    
    public void rotateOut(){
        position = position - 2;
        /*
        if(position > maxLimit){
            position = maxLimit;
        }
        if(position < minLimit){
            position = minLimit;
        }*/
    }
    public void stop() {
        position = wristEncoder.getPosition(); // Save position to hold
    }
    public void setPosition(double m_position){
        position = m_position;
    }
    public boolean atSetpoint() {
    return pidController.atSetpoint(); // Uses WPILib's built-in tolerance checking
}
    
    public void holdPosition() {
        double output = pidController.calculate(wristEncoder.getPosition(), position);
        SmartDashboard.putNumber("Wrist Position", position);
        wristMotor.set(output);
    }

    @Override
    public void periodic(){
        holdPosition();
    }
}