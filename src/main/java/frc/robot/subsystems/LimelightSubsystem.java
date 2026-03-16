package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
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
    private static final String kLimelightName = "limelight";

    // Rotation PID
    private static final double kRotP            = 0.02;
    private static final double kRotI            = 0.0;
    private static final double kRotD            = 0.001;
    private static final double kMaxRotOut       = 0.6;
    private static final double kAlignTolerance  = 1.0;  // degrees

    // Pose estimation
    private static final double kMaxTagDistanceMeters = 10.0;
    private static final double kMaxSpinRateDegPerSec = 720.0;

    // Alliance tag IDs — CHANGE to real 2026 hub tag IDs
    private static final int kRedTagID  = 10;
    private static final int kBlueTagID = 26;

    // Debounce — frames before declaring target lost
    private static final int kTargetLostThreshold = 5;

    // =====================================================
    // Hardware
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
    private double      rotationOutput   = 0.0;
    private double      distanceToTarget = 0.0;
    private boolean     hasTarget        = false;
    private int         currentTagID     = -1;
    private RawFiducial bestTag          = null;
    private int         targetLostCount  = 0;

    // =====================================================
    // Constructor
    // =====================================================
    public LimelightSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        rotPID.setTolerance(kAlignTolerance);
        rotPID.setSetpoint(0);

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
        LimelightHelpers.SetRobotOrientation(kLimelightName, yaw, 0, 0, 0, 0, 0);
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

        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);

        if (mt2 == null)                            return;
        if (mt2.tagCount == 0)                      return;
        if (mt2.avgTagDist > kMaxTagDistanceMeters) return;

        // Reject bad single tag readings
        if (mt2.tagCount == 1 && mt2.rawFiducials.length == 1) {
            if (mt2.rawFiducials[0].ambiguity > 0.9)    return;
            if (mt2.rawFiducials[0].distToCamera > 3.0) return;
        }

        // Scale trust by distance and tag count
        // Closer + more tags = smaller stdDev = more trust
        double xyStdDev = 0.5 * (mt2.avgTagDist / mt2.tagCount);
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, 9999999);

        drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);
    }

    // =====================================================
    // 3. Track target tag — rotation PID + distance
    // =====================================================
    private void updateTargetTracking() {

        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(kLimelightName);

        // Debounce — only drop target after 5 consecutive missed frames
        if (fiducials == null || fiducials.length == 0) {
            targetLostCount++;
            if (targetLostCount >= kTargetLostThreshold) {
                rotationOutput   = 0.0;
                distanceToTarget = 0.0;
                hasTarget        = false;
                currentTagID     = -1;
                bestTag          = null;
            }
            return;
        }

        // Tag seen — reset lost counter
        targetLostCount = 0;

        // Pick goal tag based on alliance
        int goalTagID = DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Red ? kRedTagID : kBlueTagID)
            .orElse(-1);

        // Prefer goal tag, fall back to first visible tag
        bestTag = null;
        for (RawFiducial tag : fiducials) {
            if (tag.id == goalTagID) {
                bestTag = tag;
                break;
            }
        }
        if (bestTag == null) bestTag = fiducials[0];

        hasTarget        = true;
        currentTagID     = bestTag.id;
        distanceToTarget = bestTag.distToCamera;

        // Rotation PID — drives txnc to 0
        double rawRot = rotPID.calculate(bestTag.txnc, 0.0);
        rotationOutput = MathUtil.clamp(rawRot, -kMaxRotOut, kMaxRotOut);
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
        SmartDashboard.putNumber ("Limelight/txnc",           bestTag != null ? bestTag.txnc : 0);
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