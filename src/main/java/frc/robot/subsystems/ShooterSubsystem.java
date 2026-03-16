package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.00017; // feedforward — most

    private static final double kRPMTolerance = 100; // within 100 RPM = ready to shoot
    private static final double kIdleRPM = 1500; //when no target present

    //Tuning
    private static final double Pt1Distance = 1.0; //meters
    private static final double Pt1RPM = 2000; //RPM

    private static final double Pt2Distance = 3.0; //meters
    private static final double Pt2RPM = 4000; //RPM

    //Clamps
    private static final double minRPM = 1500;
    private static final double maxRPM = 5000;

    private static final double kSlope = (Pt2RPM - Pt1RPM) / (Pt2Distance - Pt1Distance);
    private static final double kOffset = Pt1RPM - (kSlope * Pt1Distance);

    private final SparkFlex leftShoot;
    private final SparkFlex rightShoot;
    private final SparkClosedLoopController leftPID;
    private final SparkClosedLoopController rightPID;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final SparkFlex feed;

    private double targetRPM = 0.0;
    private double calculatedRPM = 0.0;
    private double  prevAvgRPM     = 0.0;
    private double  snapshotRPM    = 0.0;
    private boolean shotDetected   = false;
    private boolean tuningMode     = false;


  

    public ShooterSubsystem(){

        leftShoot = new SparkFlex(ShooterConstants.outLeftID, MotorType.kBrushless);
        rightShoot = new SparkFlex(ShooterConstants.outRightID, MotorType.kBrushless);

        feed = new SparkFlex(ShooterConstants.feedID, MotorType.kBrushless);
        

        // Left motor
    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast);
    leftConfig.closedLoop
        .pidf(kP, kI, kD, kF)
        .outputRange(-1, 1);

    leftShoot.configure(leftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        // Right motor — inverted so both wheels push ball same direction
    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast);
    rightConfig.closedLoop
        .pidf(kP, kI, kD, kF)
        .outputRange(-1, 1);

    rightShoot.configure(rightConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        leftEncoder = leftShoot.getEncoder();
        rightEncoder = rightShoot.getEncoder();
        leftPID = leftShoot.getClosedLoopController();
        rightPID = rightShoot.getClosedLoopController();

        
    }

    public void setRPM(double rpm){
        targetRPM = rpm;
        leftPID.setReference(rpm, ControlType.kVelocity);
        rightPID.setReference(rpm, ControlType.kVelocity);
    }

    public void setRPMFromDistance(double distanceMeters) {
        calculatedRPM = MathUtil.clamp(
            (kSlope * distanceMeters) + kOffset,
            minRPM,
            maxRPM
        );
        setRPM(calculatedRPM);
    }

   
    public void runFeed(){
        feed.set(-0.3);
    }
    public void stopFeed(){
        feed.set(0);
    }
    
    public void stop(){
        feed.stopMotor();
        targetRPM = 0.0;
        calculatedRPM = 0.0;
        leftShoot.stopMotor();
        rightShoot.stopMotor();
    }

    public void enableTuningMode()  { tuningMode = true;  }
    public void disableTuningMode() { tuningMode = false; }

    public boolean atTargetRPM() {
        if (targetRPM == 0) return false;
        return Math.abs(leftEncoder.getVelocity()    - targetRPM) < kRPMTolerance
            && Math.abs(rightEncoder.getVelocity() - targetRPM) < kRPMTolerance;
    }

    public double  getTopRPM()        { return leftEncoder.getVelocity();    }
    public double  getBottomRPM()     { return rightEncoder.getVelocity(); }
    public double  getAvgRPM()        { return (getTopRPM() + getBottomRPM()) / 2.0; }
    public double  getTargetRPM()     { return targetRPM;                   }
    public double  getCalculatedRPM() { return calculatedRPM;               }
    public double  getSnapshotRPM()   { return snapshotRPM;                 }
    public boolean shotDetected()     { return shotDetected;                }

    public void resetSnapshot() {
        snapshotRPM  = 0.0;
        shotDetected = false;
    }

    @Override
    public void periodic(){
        double avgRPM = getAvgRPM();

        // ── Tuning mode — control RPM from Shuffleboard slider ────────────
        if (tuningMode) {
            double tuningTarget = SmartDashboard.getNumber("Shooter/TuningRPM", 0.0);
            setRPM(tuningTarget);
            SmartDashboard.putNumber("Shooter/TuningRPM", tuningTarget);
        }

        // ── Shot detection — capture RPM before ball causes dip ───────────
        if (targetRPM > 0 && (prevAvgRPM - avgRPM) > 150) {
            snapshotRPM  = prevAvgRPM;
            shotDetected = true;
            System.out.printf("SHOT — RPM: %.0f%n", snapshotRPM);
        }
        prevAvgRPM = avgRPM;

        // ── Telemetry ─────────────────────────────────────────────────────
        SmartDashboard.putNumber ("Shooter/TopRPM",        getTopRPM());
        SmartDashboard.putNumber ("Shooter/BottomRPM",     getBottomRPM());
        SmartDashboard.putNumber ("Shooter/AvgRPM",        avgRPM);
        SmartDashboard.putNumber ("Shooter/TargetRPM",     targetRPM);
        SmartDashboard.putNumber ("Shooter/CalculatedRPM", calculatedRPM);
        SmartDashboard.putNumber ("Shooter/SnapshotRPM",   snapshotRPM);
        SmartDashboard.putNumber ("Shooter/TopError",      Math.abs(getTopRPM()    - targetRPM));
        SmartDashboard.putNumber ("Shooter/BottomError",   Math.abs(getBottomRPM() - targetRPM));
        SmartDashboard.putBoolean("Shooter/AtTargetRPM",   atTargetRPM());
        SmartDashboard.putBoolean("Shooter/ShotDetected",  shotDetected);
        SmartDashboard.putBoolean("Shooter/TuningMode",    tuningMode);
            }
}