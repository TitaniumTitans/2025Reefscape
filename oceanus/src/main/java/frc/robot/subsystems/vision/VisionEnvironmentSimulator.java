package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import java.util.function.Supplier;

/**
 * A class used to simulate a vision environment based on the PhotonVision VisionSystemSim This includes the cameras,
 * their simulated properties and also the april tags located around the field
 */
public class VisionEnvironmentSimulator {
    private static VisionEnvironmentSimulator instance;
    private final VisionSystemSim visionSystemSim;
    private static Supplier<Pose2d> robotPoseSupplier;

    private VisionEnvironmentSimulator() {
        visionSystemSim = new VisionSystemSim("main");
        // Additional initialization if needed
    }

    public static VisionEnvironmentSimulator getInstance() {
        if (instance == null) {
            instance = new VisionEnvironmentSimulator();
        }
        return instance;
    }

    /**
     * Add a robot pose supplier to the vision simulator to make sure the simulator knows what the camera would see
     *
     * @param actualRobotPoseSupplier supplier of the current robot pose
     */
    public void addRobotPoseSupplier(Supplier<Pose2d> actualRobotPoseSupplier) {
        robotPoseSupplier = actualRobotPoseSupplier;
    }

    /**
     * Add the april tag layout to the vision simulator
     *
     * @param aprilTagLayout the layout of the april tags around the field
     */
    public void addAprilTags(AprilTagFieldLayout aprilTagLayout) {
        visionSystemSim.addAprilTags(aprilTagLayout);
    }

    /**
     * Add a simulated camera to the vision simulator with the robot to camera transform
     *
     * @param cameraSim Simulated PhotonVision camera
     * @param robotToCamera Transform that represents the translation and rotation from robot centre to camera
     */
    public void addCamera(PhotonCameraSim cameraSim, Transform3d robotToCamera) {
        visionSystemSim.addCamera(cameraSim, robotToCamera);
    }

    /** Update the vision simulator */
    public void update() {
        if (robotPoseSupplier != null) {
            visionSystemSim.update(robotPoseSupplier.get());
        }
    }
}