package frc.robot.vision;

public class Selection extends Vision {

  public int pegNum = 1;

  public Selection() {}

  public void LeftPeg() {}

  public void RightPeg() {}

  public boolean hasVision() {
    return !super.camera.getAllUnreadResults().isEmpty();
  }

  private void GetId() {

    for (var change : camera.getAllUnreadResults()) {

      super.photonEstimator.getFieldTags().getTagPose(change.getBestTarget().getFiducialId());
    }
  }
}
