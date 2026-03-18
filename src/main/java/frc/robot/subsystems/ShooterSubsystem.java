package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.Arrays;

public class ShooterSubsystem extends SubsystemBase {

    // Constants
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.00017;

    private static final double kRPMTolerance = 100;
    private static final double kIdleRPM      = 2500;

    private static final double kMinRPM = 3500;
    private static final double kMaxRPM = 5000;

    // Hardware
    private final SparkFlex                 leftShoot;
    private final SparkFlex                 rightShoot;
    private final SparkFlex                 feed;
    private final SparkClosedLoopController leftPID;
    private final RelativeEncoder           leftEncoder;
    private final RelativeEncoder           rightEncoder;

    // Interpolating speed map — editable from Shuffleboard
    private final InterpolatingDoubleTreeMap speedMap   = new InterpolatingDoubleTreeMap();
    private double[] lastMapData = new double[0];

    private final GenericEntry SPEED_MAP_ENTRY = Shuffleboard.getTab("Shooter Tuning")
        .add("Speed Map", new double[]{  // distance(m), RPM — TUNE THESE
            2.0, 1900.0,
            3.0, 2700.0,
            4.0, 3650.0,
            5.0, 4200.0
        })
        .getEntry();

    // Firing boost — extra voltage to fight RPM dip when ball passes through
    private final GenericEntry FIRE_BOOST_VOLTAGE = Shuffleboard.getTab("Shooter Tuning")
        .add("Fire Boost Voltage", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(java.util.Map.of("min", 0, "max", 3))
        .getEntry();

    // Manual RPM override — set to -1 to use distance map
    private final GenericEntry MANUAL_RPM = Shuffleboard.getTab("Shooter Tuning")
        .add("Manual RPM Override", -1.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(java.util.Map.of("min", -1, "max", 6000))
        .getEntry();

    // State
    private double  targetRPM    = 0.0;
    private double  prevAvgRPM   = 0.0;
    private double  snapshotRPM  = 0.0;
    private boolean shotDetected = false;
    private boolean tuningMode   = false;
    private boolean feedRunning  = false;

    // Constructor
    public ShooterSubsystem() {
        leftShoot  = new SparkFlex(ShooterConstants.outLeftID,  MotorType.kBrushless);
        rightShoot = new SparkFlex(ShooterConstants.outRightID, MotorType.kBrushless);
        feed       = new SparkFlex(ShooterConstants.feedID,     MotorType.kBrushless);

        // Left — main motor with PID
        SparkFlexConfig leftConfig = new SparkFlexConfig();
        leftConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);
        leftConfig.closedLoop
            .pidf(kP, kI, kD, kF)
            .outputRange(-1, 1);
        leftShoot.configure(leftConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);

        // Right — follower mirrors left motor (inverted)
        SparkFlexConfig rightConfig = new SparkFlexConfig();
        rightConfig
            .follow(leftShoot, true) // true = inverted follow
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);
        rightShoot.configure(rightConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);

        // Feed
        SparkFlexConfig feedConfig = new SparkFlexConfig();
        feedConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(50);
        feed.configure(feedConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);

        leftEncoder  = leftShoot.getEncoder();
        rightEncoder = rightShoot.getEncoder();
        leftPID      = leftShoot.getClosedLoopController();
    }

    // Control

    /**
     * Set RPM with optional firing boost voltage
     * firingBoost = true adds extra voltage to fight RPM dip when ball fires
     */
    public void setRPM(double rpm, boolean firingBoost) {
        targetRPM = MathUtil.clamp(rpm, kMinRPM, kMaxRPM);

        double boost = firingBoost ? FIRE_BOOST_VOLTAGE.getDouble(0) : 0;

        leftPID.setReference(
            targetRPM,
            ControlType.kVelocity,

            ClosedLoopSlot.kSlot0,
            boost,
            ArbFFUnits.kVoltage
        );
    }

    public void setRPM(double rpm) { setRPM(rpm, false); }

    /**
     * Uses interpolating map to find RPM for given distance.
     * Map is editable live from Shuffleboard — no redeploy needed.
     */
    public void setRPMFromDistance(double distanceMeters) {
        // Check for manual override from Shuffleboard
        double manual = MANUAL_RPM.getDouble(-1);
        double rpm = (manual != -1) ? manual : speedMap.get(distanceMeters);
        setRPM(rpm);
    }

    public void idleSpeed()     { setRPM(kIdleRPM); }

    public void runFeed()  {
        feed.setVoltage(8.0);
        feedRunning = true;
        // Apply firing boost when feeding
    }

    public void reverseFeed() {
        feed.setVoltage(-8.0);
        feedRunning = true;
    }

    public void stopFeed() {
        feed.stopMotor();
        feedRunning = false;
    }

    public void stop() {
        targetRPM = 0.0;
        leftShoot.stopMotor();
        feed.stopMotor();
        feedRunning = false;
    }

    // Tuning
    public void enableTuningMode()  { tuningMode = true;  }
    public void disableTuningMode() { tuningMode = false; }
    public boolean isTuningMode()   { return tuningMode;  }

    public void resetSnapshot() {
        snapshotRPM  = 0.0;
        shotDetected = false;
    }

    // State
    public boolean atTargetRPM() {
        if (targetRPM == 0) return false;
        return Math.abs(getLeftRPM() - targetRPM) < kRPMTolerance;
    }

    public double  getLeftRPM()      { return leftEncoder.getVelocity();             }
    public double  getRightRPM()     { return rightEncoder.getVelocity();            }
    public double  getAvgRPM()       { return (getLeftRPM() + getRightRPM()) / 2.0;  }
    public double  getTargetRPM()    { return targetRPM;                             }
    public double  getSnapshotRPM()  { return snapshotRPM;                           }
    public boolean shotDetected()    { return shotDetected;                          }
    public boolean getFeedRunning()  { return feedRunning;                           }

    // =====================================================
    // Periodic
    // =====================================================
    @Override
    public void periodic() {
        double avgRPM = getAvgRPM();

        // Update speed map if changed in Shuffleboard
        double[] mapData = SPEED_MAP_ENTRY.getDoubleArray(new double[0]);
        if (!Arrays.equals(mapData, lastMapData)) {
            speedMap.clear();
            for (int i = 0; i < mapData.length - 1; i += 2) {
                speedMap.put(mapData[i], mapData[i + 1]);
            }
            lastMapData = mapData;
            System.out.println("Speed map updated!");
        }

        // Tuning mode — read from SmartDashboard slider
        if (tuningMode) {
            double tuningTarget = SmartDashboard.getNumber("Shooter/TuningRPM", 0.0);
            setRPM(tuningTarget);
            SmartDashboard.putNumber("Shooter/TuningRPM", tuningTarget);
        }

        // Shot detection
        if (targetRPM > 0 && (prevAvgRPM - avgRPM) > 150) {
            snapshotRPM  = prevAvgRPM;
            shotDetected = true;
            System.out.printf("SHOT — RPM: %.0f%n", snapshotRPM);
        }
        prevAvgRPM = avgRPM;

        // Telemetry
        SmartDashboard.putNumber ("Shooter/LeftRPM",      getLeftRPM());
        SmartDashboard.putNumber ("Shooter/RightRPM",     getRightRPM());
        SmartDashboard.putNumber ("Shooter/AvgRPM",       avgRPM);
        SmartDashboard.putNumber ("Shooter/TargetRPM",    targetRPM);
        SmartDashboard.putNumber ("Shooter/SnapshotRPM",  snapshotRPM);
        SmartDashboard.putNumber ("Shooter/LeftError",    Math.abs(getLeftRPM() - targetRPM));
        SmartDashboard.putBoolean("Shooter/AtTargetRPM",  atTargetRPM());
        SmartDashboard.putBoolean("Shooter/ShotDetected", shotDetected);
        SmartDashboard.putBoolean("Shooter/TuningMode",   tuningMode);
        SmartDashboard.putBoolean("Shooter/FeedRunning",  feedRunning);
    }
}