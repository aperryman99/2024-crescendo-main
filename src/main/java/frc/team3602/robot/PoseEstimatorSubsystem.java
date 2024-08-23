

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PoseEstimatorSubsystem extends SubsystemBase {
    
    private final PhotonCamera camera;
    private final SwerveDrivePoseEstimator poseEstimator;

    public PoseEstimatorSubsystem(SwerveDrivePoseEstimator poseEstimator) {
        camera = new PhotonCamera("photonvision");
        this.poseEstimator = poseEstimator;
    }

    @Override
    public void periodic() {
        // Get the latest pipeline result from the camera
        PhotonPipelineResult result = camera.getLatestResult();

        // If we have a target, we can update our robot's pose
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();

            // Estimate the robot's current pose
            Pose2d currentPose = estimatePoseFromAprilTag(target);

            // Update the pose estimator with the vision measurement
            double currentTime = Timer.getFPGATimestamp();
            poseEstimator.addVisionMeasurement(currentPose, currentTime);

            // Example: Drive to the target position 1 meter in front of the AprilTag
            Pose2d targetPosition = getTargetPosition(currentPose, 1.0);

            // Use the targetPosition for your driving logic
            // e.g., pass targetPosition to your drive subsystem for path planning
        }
    }

    private Pose2d estimatePoseFromAprilTag(PhotonTrackedTarget target) {
        // Retrieve AprilTag ID and corresponding field position
        int targetId = target.getFiducialId();

        // Assume your field layout is defined in Constants
        Pose2d tagFieldPose = Constants.APRILTAG_FIELD_LAYOUT.getPose(targetId);

        // Calculate the robot's pose on the field based on the tag pose and the camera-to-tag transformation
        Pose2d cameraPose = tagFieldPose.transformBy(target.getBestCameraToTarget().inverse());

        // Adjust camera pose to robot pose (e.g., if the camera is mounted at an offset from the robot center)
        Pose2d robotPose = cameraPose.transformBy(Constants.CAMERA_TO_ROBOT_TRANSFORM);

        return robotPose;
    }

    public Pose2d getTargetPosition(Pose2d currentPose, double distanceFromTag) {
        // Calculate the target position relative to the current pose
        // This assumes the robot should drive a certain distance in front of the AprilTag it detected

        // Get the direction the robot is facing
        Rotation2d robotHeading = currentPose.getRotation();

        // Calculate the translation in the direction the robot is facing
        Translation2d translation = new Translation2d(
            distanceFromTag * robotHeading.getCos(),
            distanceFromTag * robotHeading.getSin()
        );

        // Calculate the target position
        Pose2d targetPosition = currentPose.transformBy(
            new Pose2d(translation, new Rotation2d())
        );

        return targetPosition;
    }
}
