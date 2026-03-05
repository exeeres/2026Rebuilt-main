package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class LimelightSubsystem extends SubsystemBase {

    // =====================================================
    // Constants — TUNE THESE
    // =====================================================
    private static final String kLimelightName = "limelight"; // change if renamed

    private static final double kRotP           = 0.02;
    private static final double kRotI           = 0.0;
    private static final double kRotD           = 0.001;
    private static final double kMaxRotOut      = 0.6;
    private static final double kAlignTolerance = 1.0;  // degrees

    private static final double kCameraHeightMeters = 0.5;
    private static final double kCameraAngleDegrees = 30.0;
    private static final double kTargetHeightMeters = 1.45;

    private static final double kMaxTagDistanceMeters = 4.0;
    private static final double kMaxSpinRateDegPerSec = 720.0;

    public static final int kGoalTagID = 7; // CHANGE to your target tag

    // =====================================================
    // Subsystems
    // =====================================================
    private final CommandSwerveDrivetrain drivetrain;
    private final Field2d field = new Field2d();

    // =====================================================
    // PID
    // =====================================================
    private final PIDController rotPID = new PIDController(kRotP, kRotI, kRotD);

    // =====================================================
    // State
    // =====================================================
    private double  rotationOutput   = 0.0;
    private double  distanceToTarget = 0.0;
    private boolean hasTarget        = false;
    private int     currentTagID     = -1;

    // =====================================================
    // Constructor
    // =====================================================
    public LimelightSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        rotPID.setTolerance(kAlignTolerance);
        // No enableContinuousInput — tx is relative, not absolute

        SmartDashboard.putData("Field", field);
    }

    // =====================================================
    // Periodic
    // =====================================================
    @Override
    public void periodic() {
        updateRobotOrientation(); // MUST be first
        updatePoseEstimation();
        updateTargetTracking();
        updateTelemetry();
    }

    // =====================================================
    // 1. Send robot heading to Limelight every loop
    //    Required for MegaTag2
    // =====================================================
    private void updateRobotOrientation() {
        double yaw = drivetrain.getState().Pose.getRotation().getDegrees();

        // Officially supported method — works on all Limelight versions
        LimelightHelpers.SetRobotOrientation(
            kLimelightName,
            yaw,  // yaw degrees
            0,    // yaw rate
            0,    // pitch
            0,    // pitch rate
            0,    // roll
            0     // roll rate
        );
    }

    // =====================================================
    // 2. MegaTag2 pose fusion into drivetrain odometry
    // =====================================================
    private void updatePoseEstimation() {
        // Skip if spinning too fast — causes bad vision readings
        double spinRate = Math.abs(
            drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()
        );
        if (spinRate > kMaxSpinRateDegPerSec) return;

        // MegaTag2 — uses robot heading for field-relative pose
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);

        if (mt2 == null)           return;
        if (mt2.tagCount == 0)     return;
        if (mt2.avgTagDist > kMaxTagDistanceMeters) return;

        // Scale trust by distance and tag count
        // More tags + closer distance = smaller stdDev = more trust
        double xyStdDev = 0.5 * (mt2.avgTagDist / mt2.tagCount);

        Matrix<N3, N1> stdDevs = VecBuilder.fill(
            xyStdDev,
            xyStdDev,
            9999999  // Don't correct rotation — let Pigeon2 handle it
        );

        drivetrain.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds,
            stdDevs
        );
    }

    // =====================================================
    // 3. Track target tag — compute rotation PID + distance
    // =====================================================
    private void updateTargetTracking() {
        rotationOutput   = 0.0;
        distanceToTarget = 0.0;
        hasTarget        = false;
        currentTagID     = -1;

        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(kLimelightName);
        if (fiducials == null || fiducials.length == 0) return;

        // Prefer goal tag, fall back to closest tag
        RawFiducial bestTag = null;
        for (RawFiducial tag : fiducials) {
            if (tag.id == kGoalTagID) {
                bestTag = tag;
                break;
            }
        }
        if (bestTag == null) bestTag = fiducials[0];

        hasTarget    = true;
        currentTagID = bestTag.id;

        // tx from RawFiducial is txnc (principal pixel — most accurate)
        double tx  = bestTag.txnc;
        double raw = rotPID.calculate(tx, 0.0);
        rotationOutput = MathUtil.clamp(raw, -kMaxRotOut, kMaxRotOut);

        // Distance from camera geometry using distToCamera directly
        // LimelightHelpers gives us this for free — no ty math needed!
        distanceToTarget = bestTag.distToCamera;
    }

    // =====================================================
    // 4. Telemetry
    // =====================================================
    private void updateTelemetry() {
        SmartDashboard.putBoolean("Limelight/HasTarget",      hasTarget);
        SmartDashboard.putBoolean("Limelight/Aligned",        isAligned());
        SmartDashboard.putNumber ("Limelight/RotationOutput", rotationOutput);
        SmartDashboard.putNumber ("Limelight/DistanceMeters", distanceToTarget);
        SmartDashboard.putNumber ("Limelight/CurrentTagID",   currentTagID);
        SmartDashboard.putNumber ("Limelight/RobotYaw",
            drivetrain.getState().Pose.getRotation().getDegrees());

        field.setRobotPose(drivetrain.getState().Pose);
    }

    // =====================================================
    // Public API
    // =====================================================

    /** Rotation output for drivetrain omega (-1 to 1) */
    public double getRotationOutput() { return rotationOutput; }

    /** Distance to tracked tag in meters */
    public double getDistanceToTarget() { return distanceToTarget; }

    /** True when centered on tag within tolerance */
    public boolean isAligned() { return hasTarget && rotPID.atSetpoint(); }

    /** True when any tag is visible */
    public boolean hasTarget() { return hasTarget; }

    /** ID of currently tracked tag, -1 if none */
    public int getCurrentTagID() { return currentTagID; }

    /** Current robot pose from fused odometry */
    public Pose2d getRobotPose() { return drivetrain.getState().Pose; }
}