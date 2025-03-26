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
        do {
          if (!result.hasTargets()) {
            break;
          }

          // Go through all the targets found and discard those with an
          // ambiguity greater than 0.2.
          var targetsList = result.getTargets();
          int index = 0;
          boolean modifiedResult = false;
          while (index < targetsList.size()) {
            if (targetsList.get(index).getPoseAmbiguity() > 0.2) {
              targetsList.remove(index);
              modifiedResult = true;
            } else {
              index++;
            }
          }
          if (modifiedResult && targetsList.size() > 0) {
            // We removed at least one target from the result. Construct a new
            // Photon result with the remaining target(s).
            PhotonPipelineResult filteredResult =
                new PhotonPipelineResult(result.metadata, targetsList, Optional.empty());
            bestTimestamp = filteredResult.getTimestampSeconds();
            latestResult = Optional.of(filteredResult);
          } else if (!modifiedResult) {
            bestTimestamp = result.getTimestampSeconds();
            latestResult = Optional.of(result);
          }
        } while (false);
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
    m_visionEstimatePose = Optional.empty();
    m_target = Optional.empty();
    getLatestResult()
        .ifPresent(
            result -> {
              m_visionEstimatePose = m_cameraPoseEstimator.update(result);
              updateEstimationStdDevs(m_visionEstimatePose, result.getTargets());
              if (result.hasTargets()) {
                var targets = result.getTargets();
                double bestAmbiguity = 2.0;
                // Go through all targets seen and pick the one with the lowest ambiguity.
                // If a target has an ambiguity close enough to the best target found so far,
                // pick the target with the smallest distance from the camera as the selection
                // criteria.
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
            });
  }

  public Optional<PhotonTrackedTarget> bestTarget() {
    return m_target;
  }
}
