package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Conversions {

  /**
   * @param wheelRPS Wheel Velocity: (in Rotations per Second)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Velocity: (in Meters per Second)
   */
  public static double RPSToMPS(double wheelRPS, double circumference) {
    double wheelMPS = wheelRPS * circumference;
    return wheelMPS;
  }

  /**
   * @param wheelMPS Wheel Velocity: (in Meters per Second)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Velocity: (in Rotations per Second)
   */
  public static double MPSToRPS(double wheelMPS, double circumference) {
    double wheelRPS = wheelMPS / circumference;
    return wheelRPS;
  }

  /**
   * @param wheelRotations Wheel Position: (in Rotations)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Distance: (in Meters)
   */
  public static double rotationsToMeters(double wheelRotations, double circumference) {
    double wheelMeters = wheelRotations * circumference;
    return wheelMeters;
  }

  /**
   * @param wheelMeters Wheel Distance: (in Meters)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Position: (in Rotations)
   */
  public static double metersToRotations(double wheelMeters, double circumference) {
    double wheelRotations = wheelMeters / circumference;
    return wheelRotations;
  }

  /**
   * @brief Converts a Rotation3d to a Rotation2d by removing the Z component
   * @param Rotation3d
   * @return Rotation2d
   */
  public static Rotation2d Rotation2dToRotation3d(Rotation3d rotation) {
    return new Rotation2d(rotation.getX(), rotation.getY());
  }

  /**
   * @brief Converts a Pose3d to a Pose2d by removing the Z component
   * @param pose Pose3d
   * @return Pose2d
   */
  public static Pose2d Pose3dToPose2d(Pose3d pose) {
    return new Pose2d(
        pose.getTranslation().getX(),
        pose.getTranslation().getY(),
        Rotation2dToRotation3d(pose.getRotation()));
  }
}
