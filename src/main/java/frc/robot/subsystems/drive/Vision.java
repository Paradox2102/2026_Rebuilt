package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.awt.Desktop;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

public class Vision {
  private static AprilTagFieldLayout m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public VisionSystemSim m_visionSim;

  private Supplier<Pose2d> m_currentPose;

  private Field2d m_field2d;

  public Vision(Supplier<Pose2d> m_currentPose, Field2d field) {
    this.m_currentPose = m_currentPose;
    this.m_field2d = field;

    if (Robot.isSimulation()) {
      m_visionSim = new VisionSystemSim("Vision");
      m_visionSim.addAprilTags(m_fieldLayout);

      for (Cameras c : Cameras.values()) {
        c.addToVisionSim(m_visionSim);
      }

      openSimCameraViews();
    }
  }

  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = m_fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + m_fieldLayout.toString());
    }
  }

  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent() && !RobotBase.isReal()) {
      /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for
       * factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate
       * pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the
       * simulator when updating the vision simulation during the simulation.
       */
      // m_visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    for (Cameras camera : Cameras.values()) {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent()) {
        var pose = poseEst.get();
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
            pose.timestampSeconds,
            camera.curStdDevs);
      }
    }

  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation()) {
      Field2d debugField = m_visionSim.getDebugField();
      poseEst.ifPresentOrElse(
          est -> debugField
              .getObject("VisionEstimation")
              .setPose(est.estimatedPose.toPose2d()),
          () -> {
            debugField.getObject("VisionEstimation").setPoses();
          });
    }
    return poseEst;
  }

  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;

  }

  public VisionSystemSim getVisionSim() {
    return m_visionSim;
  }

  private void openSimCameraViews() {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
      try
      {
      Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
      Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
      } catch (IOException | URISyntaxException e)
      {
      e.printStackTrace();
      }
    }
  }

  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = m_fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(m_currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  public void updateVisionField() {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (m_fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = m_fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    m_field2d.getObject("tracked targets").setPoses(poses);
  }

  public enum Cameras {
    FRONT_LEFT(
        "front left", VisionConstants.k_flRotation, VisionConstants.k_flTranslation, VisionConstants.k_visionBaseSDev, VisionConstants.k_multiTagSDev),
    FRONT_RIGHT(
        "front right", VisionConstants.k_frRotation, VisionConstants.k_frTranslation, VisionConstants.k_visionBaseSDev, VisionConstants.k_multiTagSDev),
    SIDE_LEFT(
        "side left", VisionConstants.k_slRotation, VisionConstants.k_slTranslation, VisionConstants.k_visionBaseSDev, VisionConstants.k_multiTagSDev),
    SIDE_RIGHT(
        "side right", VisionConstants.k_srRotation, VisionConstants.k_srTranslation, VisionConstants.k_visionBaseSDev, VisionConstants.k_multiTagSDev);

    public final PhotonCamera camera;

    public final PhotonPoseEstimator poseEstimator;

    private final Matrix<N3, N1> singleTagStdDevs;

    private final Matrix<N3, N1> multiTagStdDevs;

    private final Transform3d robotToCamera;

    public Matrix<N3, N1> curStdDevs;

    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    public PhotonCameraSim cameraSim;

    public List<PhotonPipelineResult> resultsList = new ArrayList<>();

    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    private boolean filterResult = false;

    private Set<Integer> ignoreSet = new HashSet<>();

    private StructPublisher<Pose3d> cameraPosePublisher = NetworkTableInstance.getDefault().getStructTopic(name() + " camera est pose", Pose3d.struct).publish();

    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
        Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
      camera = new PhotonCamera(name);

      robotToCamera = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(Vision.m_fieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCamera);

      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      for(int ignore : VisionConstants.ignoreTagList){
        ignoreSet.add(ignore);
      }

      if (Robot.isSimulation()) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in
        // pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop
        // rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    public void addToVisionSim(VisionSystemSim systemSim) {
      if (Robot.isSimulation()) {
        systemSim.addCamera(cameraSim, robotToCamera);
      }
    }

    public Optional<PhotonPipelineResult> getBestResult() {
      if (resultsList.isEmpty()) {
        return Optional.empty();
      }

    PhotonPipelineResult bestResult = resultsList.get(0);
      double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList) {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
          bestResult = result;
          amiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    public Optional<PhotonPipelineResult> getLatestResult() {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      updateUnreadResults();
      if(estimatedRobotPose.isPresent() && !filterResult){
        cameraPosePublisher.set(estimatedRobotPose.get().estimatedPose);
      } else {
        cameraPosePublisher.set(new Pose3d());
      }
      return filterResult ? Optional.empty() : estimatedRobotPose;
    }

    private void updateUnreadResults() {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      for (PhotonPipelineResult result : resultsList) {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }

      resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
      lastReadTimestamp = currentTimestamp;
      resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
        return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
      });
      if (!resultsList.isEmpty()) {
        updateEstimatedGlobalPose();
      }
    }

    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList) {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      filterResult = false;
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else {
        // Pose present. Start running Heuristic
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;
        double minDist = Double.MAX_VALUE;

        // Precalculation - see how many tags we found, and calculate an
        // average-distance metric
        for (var tgt : targets) {
          int id = tgt.getFiducialId();
          var tagPose = poseEstimator.getFieldTags().getTagPose(id);
          if(ignoreSet.contains(id)){
            filterResult = true;
          }
          if (tagPose.isEmpty()) {
            continue;
          }
          numTags++;
          double dist = tagPose
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());

          if(dist < minDist){
            minDist = dist;
          }

          avgDist += dist;
        }

        if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          if(minDist > 4){
            filterResult = true;
          }

          avgDist /= numTags;
          SmartDashboard.putNumber(name() + " avg tag distance", avgDist);
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
          }

          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 15));
          curStdDevs = estStdDevs;
        }
      }
    }
  }
}