// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.*;
import frc.robot.vision.Selection;
import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAlignCmd extends SequentialCommandGroup {

  private Swerve swerve;
  private Selection selector;
  private double desiredDistFromTag = 1;
  private Pose2d desiredPose = new Pose2d();
  private Trajectory trajectory;
  private SwerveControllerCommand swerveControllerCommand;
  private boolean hasValidPose = false;

  /** Creates a new AlgaeAlignCmd. */
  public AlgaeAlignCmd(Swerve swerve, Selection selector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.selector = selector;
    addRequirements(swerve);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(0.5, 0.5)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Swerve.swerveKinematics);

    var thetaController =
        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    System.out.println(" this fucking bitch works, as it should cuz im bouta lose my nerve");
    if (hasValidPose) {

      addCommands(
          new InstantCommand(
              () -> {
                if (selector.getLockId() != 0) {
                  hasValidPose = true;
                  var desiredTranslation =
                      new Translation2d(
                          selector.GetTagTranslation().getX()
                              + (Math.cos(selector.GetTagYaw()) * desiredDistFromTag),
                          selector.GetTagTranslation().getY()
                              + (Math.sin(selector.GetTagYaw()) * desiredDistFromTag));
                  var desiredRot = selector.GetTagYaw() + Math.toRadians(180);

                  desiredPose = new Pose2d(desiredTranslation, new Rotation2d(desiredRot));
                } else {
                  hasValidPose = false;
                }
              }),
          new InstantCommand(
                  () ->
                      trajectory =
                          TrajectoryGenerator.generateTrajectory(
                              // Start at the origin facing the +X direction
                              swerve.getPose(),
                              // Pass through these two interior waypoints, making an 's' curve path
                              List.of(),
                              // End 3 meters straight ahead of where we started, facing forward
                              desiredPose,
                              config))
              .unless(() -> !hasValidPose),
          new PrintCommand(desiredPose.getX() + " " + desiredPose.getY())
              .unless(() -> !hasValidPose),
          new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose()))
              .unless(() -> !hasValidPose),
          new SwerveControllerCommand(
                  trajectory,
                  swerve::getPose,
                  Constants.Swerve.swerveKinematics,

                  // Position controllers
                  new PIDController(5.0, 0, 0),
                  new PIDController(5.0, 0, 0),
                  thetaController,
                  swerve::setModuleStates,
                  swerve)
              .unless(() -> !hasValidPose),
          new InstantCommand(() -> swerve.drive(new Translation2d(0, 0), 0, false, false)));
    }
  }
}
