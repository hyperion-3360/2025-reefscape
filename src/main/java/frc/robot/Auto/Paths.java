// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Paths {

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(new Pose2d(0, 0, new Rotation2d()));

    PathConstraints constraints = new PathConstraints(1.0, 1.0, 1, 2);

    PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, Rotation2d.fromDegrees(0)));

}
