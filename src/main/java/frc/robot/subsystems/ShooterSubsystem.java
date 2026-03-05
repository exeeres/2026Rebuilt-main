package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    
    SparkFlex leftShoot;
    SparkFlex rightShoot;
    SparkFlex feed;

    double startTime;
    

    public ShooterSubsystem(){

        leftShoot = new SparkFlex(ShooterConstants.outLeftID, MotorType.kBrushless);
        rightShoot = new SparkFlex(ShooterConstants.outRightID, MotorType.kBrushless);

        feed = new SparkFlex(ShooterConstants.feedID, MotorType.kBrushless);

    }

    public void speedUp(double speed, double speedUpTime){
        startTime = Timer.getFPGATimestamp();
        
        while(Timer.getFPGATimestamp() - startTime < speedUpTime){
            leftShoot.set(-speed); // clockwise
            rightShoot.set(speed); //counterclockwise
        }
    }

    public void fire(double fireSpeed, double shootTime){
        startTime = Timer.getFPGATimestamp();
        while(Timer.getFPGATimestamp() - startTime < shootTime){
        feed.set(-0.3);

        leftShoot.set(-fireSpeed);
        rightShoot.set(fireSpeed);
        }
    }
    
    public void stop(){
        feed.set(0);

        leftShoot.set(0);
        rightShoot.set(0);
    }
}
