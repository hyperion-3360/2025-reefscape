// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.Selection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefAlignCmd extends Command {
  /** Creates a new ReefAlignCmd. */
  public enum direction {
    Left,
    Right
  }

  private Swerve swerve;
  private Selection selector;
  private direction direction;
  private Pose2d desiredPose;
  private double pegDistFromTag = 0.0;
  private Trajectory trajectory;
  private SwerveControllerCommand swerveControllerCommand;
  private boolean isFinished = false;

  public ReefAlignCmd(direction direction, Selection selector, Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.selector = selector;
    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    var desiredTranslation = new Translation2d();
    var desiredRot = 0.0;
    switch (direction) {
      case Left:
        desiredTranslation = // new Translation2d(2.66, 4.01);
            new Translation2d(
                selector.GetTagTranslation().getX() + (Math.sin(selector.GetTagYaw()) * pegDistFromTag),
                selector.GetTagTranslation().getY() - (Math.sin(selector.GetTagYaw()) * pegDistFromTag));
        desiredRot = selector.GetTagYaw() + Math.toRadians(180);
        desiredPose = new Pose2d(desiredTranslation, new Rotation2d(desiredRot));

        break;

      case Right:
        desiredTranslation = // new Translation2d(2.66, 4.01);
            new Translation2d(
              selector.GetTagTranslation().getX() - (Math.cos(selector.GetTagYaw()) * pegDistFromTag),
              selector.GetTagTranslation().getY() + (Math.sin(selector.GetTagYaw()) * pegDistFromTag));
        desiredRot = selector.GetTagYaw() + Math.toRadians(180);
        desiredPose = new Pose2d(desiredTranslation, new Rotation2d(desiredRot));


        break;

      default:
        break;
    }

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(0.5, 0.5)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Swerve.swerveKinematics);

    // An example trajectory to follow. All units in meters.
    trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            swerve.getPose(),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            desiredPose,
            config);

    var thetaController =
        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            swerve::getPose, // Functional interface to feed supplier
            Constants.Swerve.swerveKinematics,

            // Position controllers
            new PIDController(5.0, 0, 0),
            new PIDController(5.0, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Commands.sequence(
        new PrintCommand("hello again"),
        new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> swerve.drive(new Translation2d(0, 0), 0, false, false)),
        new InstantCommand(() -> isFinished = true));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
