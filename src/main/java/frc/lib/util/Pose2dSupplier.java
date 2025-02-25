package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;

@FunctionalInterface
public interface Pose2dSupplier {

  /**
   * Gets a result.
   *
   * @return a result
   */
  Pose2d getAsPose2d();
}
