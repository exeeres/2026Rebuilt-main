package frc.robot.commands.lime;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.PoseEstimate;
import limelight.networktables.target.AprilTagFiducial;

public class Lime extends SubsystemBase {

    // Limelight instance (name must match NT table name)
    private final Limelight limelight = new Limelight("limelight");

    // Make gains easy to find/tune
    private static final double kP = 0.02;
    private static final double kI = 0.0;
    private static final double kD = 0.001;
    private static final double kMaxOutput = 1.0;

    // Rotation PID (TUNE THIS ON ROBOT)
    private final PIDController turnPID = new PIDController(kP, kI, kD);

    // Output sent to drivetrain
    private double rotationOutput = 0;

    // Drivetrain pose estimator (passed in from drivetrain)
    private final edu.wpi.first.math.estimator.SwerveDrivePoseEstimator poseEstimator;

    // Create the limelight pose estimator once
    private final LimelightPoseEstimator visionPoseEstimator;

    public Lime(edu.wpi.first.math.estimator.SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;

        // Configure Limelight ONCE
        limelight.getSettings()
                .withCameraOffset(Pose3d.kZero)
                .save();

        // Stop turning when within 1 degree
        turnPID.setTolerance(1.0);

        // Enable continuous angle wrapping for heading errors (-180..180)
        turnPID.enableContinuousInput(-180.0, 180.0); // remove if there is strange behavior

        // create pose estimator once
        visionPoseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
    }

    @Override
    public void periodic() {

        // ===============================
        // 1. MegaTag2 Pose Integration
        // ===============================
        Optional<PoseEstimate> visionEstimate = visionPoseEstimator.getPoseEstimate();
        System.out.println("Vision estimate: " + visionEstimate); // !!!!!!!!!!!!!!!!!!!!!!!!
        visionEstimate.ifPresent(pose -> {
            poseEstimator.addVisionMeasurement(
                    pose.pose.toPose2d(),
                    pose.timestampSeconds
            );//hiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
        });

        // ===============================
        // 2. AprilTag Auto-Align
        // ===============================
        rotationOutput = 0;

        Optional<LimelightResults> resultsOpt = limelight.getLatestResults();

        resultsOpt.ifPresent(results -> {
            // NOTE: Your library uses targets_Fiducial (singular)
            if (results.targets_Fiducials != null && results.targets_Fiducials.length > 0) {
                AprilTagFiducial tag = results.targets_Fiducials[0];
                System.out.println("Tag/index 0 of target_Fiducials: " + tag);
                double tx = tag.tx;          // horizontal error (degrees)
                int id = (int) tag.fiducialID;     // tag number

                // Optional: filter for specific goal tag
                // if (id == YOUR_GOAL_TAG_ID) {
                double raw = turnPID.calculate(tx, 0.0);
                rotationOutput = MathUtil.clamp(raw, -kMaxOutput, kMaxOutput);
                // }
            }
        });

        // Telemetry for tuning/debugging
        SmartDashboard.putBoolean("Lime/HasTarget", hasTarget());
        SmartDashboard.putBoolean("Lime/Aligned", isAligned());
        SmartDashboard.putNumber("Lime/RotationOutput", rotationOutput);
    }

    // ===============================
    // Public Methods
    // ===============================

    /** Rotation command for drivetrain */
    public double getRotationOutput() {
        return rotationOutput;
    }

    /** True when robot is aimed at tag */
    public boolean isAligned() {
        return turnPID.atSetpoint();
    }

    /** Do we currently see a tag? */
    public boolean hasTarget() {
        return limelight.getLatestResults()
                .map(r -> r.targets_Fiducials != null && r.targets_Fiducials.length > 0)
                .orElse(false);
    }
}
