package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PoseSubsystem extends SubsystemBase {
    // Camera configuration constants
    private static final String CAMERA_LEFT_NAME = "left_cam";
    private static final Transform3d CAMERA_LEFT_TRANSFORM = new Transform3d(
            new Translation3d(0.5, 0.0, 0.45),
            new Rotation3d(0, 0, Math.toRadians(22.5)));

    private static final String CAMERA_RIGHT_NAME = "right_cam";
    private static final Transform3d CAMERA_RIGHT_TRANSFORM = new Transform3d(
            new Translation3d(0.5, 0.0, 0.45),
            new Rotation3d(0, 0, Math.toRadians(-22.5)));

    private final DifferentialDrivePoseEstimator poseEstimator;

    // Vision components
    private final AprilTagFieldLayout fieldLayout;
    private final List<VisionSource> visionSources;

    // Field visualization
    private final Field2d field;

    // Gyro interface
    private final GyroIO gyro;

    // Wheel encoder interface
    private final EncoderIO encoders;

    /**
     * Standard deviations for odometry (encoder + gyro)
     * [x, y, heading] in meters and radians
     */
    private static final double ODOMETRY_X_STD_DEV = 0.05;
    private static final double ODOMETRY_Y_STD_DEV = 0.05;
    private static final double ODOMETRY_THETA_STD_DEV = 0.005;

    /**
     * Baseline standard deviations for vision measurements
     * These are multiplied by distance and tag count factors
     * [x, y, heading] in meters and radians
     */
    private static final double VISION_X_STD_DEV = 0.7;
    private static final double VISION_Y_STD_DEV = 0.7;
    private static final double VISION_THETA_STD_DEV = 0.9;

    public PoseSubsystem( DriveSubsystem ds) {
        this(ds.getDifferentialDriveKinematics(), ds, ds);
    }

    public PoseSubsystem(
            DifferentialDriveKinematics kinematics,
            GyroIO gyro,
            EncoderIO encoders) {

        this.gyro = gyro;
        this.encoders = encoders;

        // Load field layout
        this.fieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

        // Initialize pose estimator with odometry standard deviations
        poseEstimator = new DifferentialDrivePoseEstimator(
                kinematics,
                gyro.getRotation2d(),
                encoders.getLeftPosition(),
                encoders.getRightPosition(),
                new Pose2d(),
                VecBuilder.fill(ODOMETRY_X_STD_DEV, ODOMETRY_Y_STD_DEV, ODOMETRY_THETA_STD_DEV),
                VecBuilder.fill(VISION_X_STD_DEV, VISION_Y_STD_DEV, VISION_THETA_STD_DEV));

        // Initialize vision sources
        visionSources = new ArrayList<>();
        visionSources.add(new VisionSource(CAMERA_LEFT_NAME, CAMERA_LEFT_TRANSFORM));
        visionSources.add(new VisionSource(CAMERA_RIGHT_NAME, CAMERA_RIGHT_TRANSFORM));

        // Field visualization
        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    /**
     * Updates the pose estimator with current sensor readings
     * Call this periodically (e.g., in periodic())
     */
    public void update() {
        // Update odometry with encoder and gyro data
        poseEstimator.update(
                gyro.getRotation2d(),
                encoders.getLeftPosition(),
                encoders.getRightPosition());

        // Process vision updates from all cameras
        for (VisionSource source : visionSources) {
            source.update().ifPresent(this::addVisionMeasurement);
        }

        // Update field visualization
        field.setRobotPose(getCurrentPose());

        // Publish telemetry
        publishTelemetry();
    }

    /**
     * Adds a vision measurement with dynamic standard deviation adjustment
     */
    private void addVisionMeasurement(EstimatedRobotPose estimate) {
        // Calculate dynamic standard deviations based on factors
        double[] dynamicStdDevs = calculateVisionStdDevs(estimate);

        poseEstimator.addVisionMeasurement(
                estimate.estimatedPose.toPose2d(),
                estimate.timestampSeconds,
                VecBuilder.fill(dynamicStdDevs[0], dynamicStdDevs[1], dynamicStdDevs[2]));
    }

    /**
     * Calculates vision standard deviations based on distance and tag count
     * More tags and closer distance = lower std dev (more trust)
     */
    private double[] calculateVisionStdDevs(EstimatedRobotPose estimate) {
        int tagCount = estimate.targetsUsed.size();
        double avgDistance = estimate.targetsUsed.stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(4.0);

        // Increase std dev with distance (less trust when far)
        double distanceFactor = Math.pow(avgDistance, 2) / 4.0;

        // Decrease std dev with more tags (more trust with multiple tags)
        double tagFactor = 1.0 / tagCount;

        double combinedFactor = distanceFactor * tagFactor;

        return new double[] {
                VISION_X_STD_DEV * combinedFactor,
                VISION_Y_STD_DEV * combinedFactor,
                VISION_THETA_STD_DEV * combinedFactor
        };
    }

    /**
     * Gets the current estimated pose
     */
    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the pose to a known position
     */
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
                gyro.getRotation2d(),
                encoders.getLeftPosition(),
                encoders.getRightPosition(),
                pose);
    }

    private void publishTelemetry() {
        Pose2d pose = getCurrentPose();
        SmartDashboard.putNumber("Pose/X", pose.getX());
        SmartDashboard.putNumber("Pose/Y", pose.getY());
        SmartDashboard.putNumber("Pose/Rotation", pose.getRotation().getDegrees());

        for (int i = 0; i < visionSources.size(); i++) {
            VisionSource source = visionSources.get(i);
            SmartDashboard.putBoolean("Vision/Cam" + i + "/HasTarget", source.hasTargets());
        }
    }

    // ========== HELPER CLASSES ==========

    /**
     * Wrapper for a single vision camera and its pose estimator
     */
    private class VisionSource {
        private final PhotonCamera camera;
        private final PhotonPoseEstimator poseEstimator;

        public VisionSource(String cameraName, Transform3d robotToCamera) {
            this.camera = new PhotonCamera(cameraName);
            this.poseEstimator = new PhotonPoseEstimator(
                    fieldLayout,
                    robotToCamera);
        }

        public Optional<EstimatedRobotPose> update() {
            PhotonPipelineResult result = camera.getLatestResult();

            if (!result.hasTargets()) {
                return Optional.empty();
            }

            // Try coprocessor multi-tag first (requires PhotonVision config)
            Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(result);

            // Fallback to lowest ambiguity if multi-tag not available
            if (estimate.isEmpty()) {
                estimate = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            return estimate;
        }

        public boolean hasTargets() {
            return camera.getLatestResult().hasTargets();
        }
    }

    /**
     * Interface for gyro access - implement this for your gyro
     */
    public interface GyroIO {
        Rotation2d getRotation2d();
    }

    /**
     * Interface for encoder access - implement this for your encoders
     */
    public interface EncoderIO {
        double getLeftPosition(); // meters

        double getRightPosition(); // meters

        double getLeftVelocity(); // meters/sec

        double getRightVelocity(); // meters/sec
    }
}