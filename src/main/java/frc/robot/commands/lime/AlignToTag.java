package frc.robot.commands.lime;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class AlignToTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightSubsystem limelight;

    // Joystick inputs passed in as suppliers so they're read live each loop
    private final DoubleSupplier translationX; // left stick Y (forward/back)
    private final DoubleSupplier translationY; // left stick X (strafe)

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private static final double MAX_SPEED        = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);       // m/s — match your TunerConstants
    private static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // rad/s
    private static final double DEADBAND         = 0.1;       // joystick deadband

    /**
     * Allows full driver translation control while limelight
     * overrides rotation to align to the AprilTag.
     *
     * @param drivetrain    Swerve drivetrain subsystem
     * @param limelight     LimelightSubsystem (already has PID inside)
     * @param translationX  Supplier for forward/back joystick axis
     * @param translationY  Supplier for strafe joystick axis
     */
    public AlignToTag(
            CommandSwerveDrivetrain drivetrain,
            LimelightSubsystem limelight,
            DoubleSupplier translationX,
            DoubleSupplier translationY) {

        this.drivetrain   = drivetrain;
        this.limelight    = limelight;
        this.translationX = translationX;
        this.translationY = translationY;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // ── 1. Read joystick translation (driver still controls movement) ──
        double xSpeed = applyDeadband(translationX.getAsDouble()); // forward/back
        double ySpeed = applyDeadband(translationY.getAsDouble()); // strafe

        // ── 2. Get rotation from limelight subsystem PID ──────────────────
        if (!limelight.hasTarget()) {
            // No tag visible — give driver full control including rotation
            // (or hold rotation still — your choice, see note below)
            drivetrain.applyRequest(() ->
                drive.withVelocityX(xSpeed * MAX_SPEED)
                     .withVelocityY(ySpeed * MAX_SPEED)
                     .withRotationalRate(0)
            ).execute();
            return;
        }

        double rotOutput = limelight.getRotationOutput();

        // ── 3. Driver moves, limelight steers ─────────────────────────────
        drivetrain.applyRequest(() ->
            drive.withVelocityX(xSpeed * MAX_SPEED)
                 .withVelocityY(ySpeed * MAX_SPEED)
                 .withRotationalRate(rotOutput * MAX_ANGULAR_RATE)
        ).execute();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until button released
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> brake).execute();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }
}