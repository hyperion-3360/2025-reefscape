package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionCamera extends SubsystemBase {

  private final PhotonCamera m_cameraInstance;

  private final PhotonPoseEstimator m_cameraPoseEstimator;

  private Matrix<N3, N1> m_curStdDevs;

  private Matrix<N3, N1> m_singleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private Matrix<N3, N1> m_multiTagStdDevs = VecBuilder.fill(3, 3, 6);

  private double m_latestTimestamp = 0;

  private Optional<EstimatedRobotPose> m_visionEstimatePose = Optional.empty();
  private Optional<PhotonTrackedTarget> m_target = Optional.empty();

  public double distanceFactor = 0;
  public double ambiguityFactor = 0;

  public VisionCamera(
      String cameraName, Transform3d robotToCam, double distanceFactor, double ambiguityFactor) {
    m_cameraInstance = new PhotonCamera(cameraName);
    m_cameraPoseEstimator =
        new PhotonPoseEstimator(Constants.tagLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCam);
    this.distanceFactor = distanceFactor;
    this.ambiguityFactor = ambiguityFactor;
  }

  public boolean isActive() {
    return m_cameraInstance.isConnected();
  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

    if (estimatedPose.isEmpty()) {
      m_curStdDevs = m_singleTagStdDevs;
    } else {

      // pose preswnt, start running heuristic
      var estStdDevs = m_singleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // precalc (how mny tags, avg dist metric)
      for (var tgt : targets) {
        var tagPose = m_cameraPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // no visiblw tags default to single tag
        m_curStdDevs = m_singleTagStdDevs;
      } else {
        // more tags, run full heuristic
        avgDist /= numTags;

        // decrase std devs if multiple visible
        if (numTags > 1) {
          estStdDevs = m_multiTagStdDevs;
        }

        // increase std devs based on "avg" dist
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        m_curStdDevs = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
    return m_curStdDevs;
  }

  Optional<PhotonPipelineResult> getLatestResult() {
    Optional<PhotonPipelineResult> latestResult = Optional.empty();
    var bestTimestamp = m_latestTimestamp;

    for (var result : m_cameraInstance.getAllUnreadResults()) {
      if (result.getTimestampSeconds() > bestTimestamp) {
        bestTimestamp = result.getTimestampSeconds();
        latestResult = Optional.of(result);
      }
    }

    if (!latestResult.isEmpty()) {
      m_latestTimestamp = bestTimestamp;
    }

    return latestResult;
  }

  public double getTimestampSeconds() {
    return m_latestTimestamp;
  }

  public Optional<EstimatedRobotPose> getVisionEstimatePose() {
    return m_visionEstimatePose;
  }

  public void updateEstimatedPose() {
    getLatestResult()
        .ifPresentOrElse(
            result -> {
              if (isGoodResult(result)) {
                m_visionEstimatePose = m_cameraPoseEstimator.update(result);
                updateEstimationStdDevs(m_visionEstimatePose, result.getTargets());
                if (result.hasTargets()) {
                  var targets = result.getTargets();
                  double bestAmbiguity = 2.0;
                  for (var target : targets) {
                    if ((target.poseAmbiguity < bestAmbiguity)
                        && Math.abs(target.poseAmbiguity - bestAmbiguity) > 0.05) {
                      bestAmbiguity = target.poseAmbiguity;
                      m_target = Optional.of(target);
                    } else if (Math.abs(target.poseAmbiguity - bestAmbiguity) < 0.05) {
                      if (target.getBestCameraToTarget().getTranslation().getNorm()
                          < m_target.get().getBestCameraToTarget().getTranslation().getNorm()) {
                        m_target = Optional.of(target);
                      }
                    }
                  }
                }
              }
            },
            () -> {
              m_visionEstimatePose = Optional.empty();
              m_target = Optional.empty();
            });
  }

  public Optional<PhotonTrackedTarget> bestTarget() {
    return m_target;
  }

  private boolean isGoodResult(PhotonPipelineResult result) {
    boolean rc = true;
    do {
      if (!result.hasTargets()) {
        rc = false;
        break;
      }

      for (var target : result.getTargets()) {
        if (target.getPoseAmbiguity() > 0.2) {
          rc = false;
          break;
        }
      }

    } while (false);

    return rc;
  }
}
