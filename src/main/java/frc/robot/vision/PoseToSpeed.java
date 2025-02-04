package frc.robot.vision;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface PoseToSpeed {
        /**
     * Converts a target {@code targetTrajectoryState} and current {@code Pose2d} into an appropriate {@code ChassisSpeeds}
     *
     * @param currentPose the current pose of the robot
     * @param targetTrajectoryState the desired position and speeds of the robot
     * @return the {@code ChassisSpeeds} to get from the current pose to the target pose
     */
    ChassisSpeeds getTargetSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetTrajectoryState);

    /**
     * Converts a target and current {@code Pose2d} into an appropriate {@code ChassisSpeeds}
     *
     * @param currentPose the current pose of the robot
     * @param targetPose the desired position of the robot
     * @return the {@code ChassisSpeeds} to get from the current pose to the target pose
     */
    ChassisSpeeds getTargetSpeeds(Pose2d currentPose, Pose2d targetPose);
}
