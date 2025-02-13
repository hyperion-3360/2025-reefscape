package frc.robot.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Selection extends Vision {

  Swerve swerve;
  List<Integer> reefPegTag = new ArrayList<Integer>();
  int lockID = -1;
  PhotonTrackedTarget trackedTarget;
  // field units are in meters, so we want to be approx 1 meter from target
  double desiredDistFromTag = 1;
  double orientationMultipleY = 0;
  // TODO: this is an abitrary value, to be measured
  double pegDistFromTag = 0.2;


  Pose2d currentPose = new Pose2d();

  double kp = 0.03;
  double ki = 0;
  double kd = 0;
  PIDController m_pid = new PIDController(kp, ki, kd);
  Pose2d desiredPose = new Pose2d();

  private boolean hasAlignedAlgae = false;

  public enum direction {
    left,
    right
  }

  public Selection(Swerve swerve) {
    this.swerve = swerve;

    var alliance = DriverStation.getAlliance().get();
    if (alliance == Alliance.Blue) {
      reefPegTag.clear();
      reefPegTag.add(22);
      reefPegTag.add(21);
      reefPegTag.add(20);
      reefPegTag.add(19);
      reefPegTag.add(18);
      reefPegTag.add(17);

    } else if (alliance == Alliance.Red) {
      reefPegTag.clear();
      reefPegTag.add(6);
      reefPegTag.add(7);
      reefPegTag.add(8);
      reefPegTag.add(9);
      reefPegTag.add(10);
      reefPegTag.add(11);

    } else {
      reefPegTag.clear();
    }
  }

  @Override
  public void periodic() {
    currentPose = swerve.getPose();
    setLockTarget();
    SmartDashboard.putNumber("lock id", lockID);

    if (lockID != 0 && !hasAlignedAlgae) {
      var desiredTranslation =
          new Translation2d(
              GetTagTranslation().getX() + (Math.cos(GetYaw()) * desiredDistFromTag),
              GetTagTranslation().getY() + (Math.sin(GetYaw()) * desiredDistFromTag));
      var desiredRot = GetYaw() + Math.toRadians(180);
      SmartDashboard.putNumber("desired x", desiredTranslation.getX());
      SmartDashboard.putNumber("desired y", desiredTranslation.getY());
      SmartDashboard.putNumber("desired roation", desiredRot);

      desiredPose = new Pose2d(desiredTranslation, new Rotation2d(desiredRot));
    }
    // System.out.println(currentPose);
    // System.out.println(lockID);
  }

  public Command MovePeg(direction direction) {
    // will have to do trigo, but make sure align works first.
    var desiredTranslation = new Translation2d();

    switch (direction) {
      case left:
        desiredTranslation = // new Translation2d(2.66, 4.01);
            new Translation2d(
                GetTagTranslation().getX() + (Math.sin(GetYaw()) * pegDistFromTag),
                GetTagTranslation().getY() - (Math.sin(GetYaw()) * pegDistFromTag));
        desiredPose = new Pose2d(desiredTranslation, new Rotation2d());

        break;

      case right:
        desiredTranslation = // new Translation2d(2.66, 4.01);
            new Translation2d(
                GetTagTranslation().getX() - (Math.cos(GetYaw()) * desiredDistFromTag),
                GetTagTranslation().getY() + (Math.sin(GetYaw()) * desiredDistFromTag));
        desiredPose = new Pose2d(desiredTranslation, new Rotation2d());


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
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            currentPose,
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            desiredPose,
            config);

    var thetaController =
        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
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

    return Commands.sequence(
        new PrintCommand("hello again"),
        new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> swerve.drive(new Translation2d(0, 0), 0, false, false)));
  }

  private void setLockTarget() {

    for (var change : camera.getAllUnreadResults()) {

      if (change.hasTargets()) {

        trackedTarget = change.getBestTarget();
        lockID = trackedTarget.fiducialId;
      } else {
        lockID = 0;
      }
    }
  }

  // will probably refactor
  public Command Align() {

    if (lockID != 0) {

      // Create config for trajectory
      TrajectoryConfig config =
          new TrajectoryConfig(0.5, 0.5)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(Constants.Swerve.swerveKinematics);

      // An example trajectory to follow. All units in meters.
      Trajectory trajectory =
          TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              currentPose,
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(),
              // End 3 meters straight ahead of where we started, facing forward
              desiredPose,
              config);

      var thetaController =
          new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand =
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

      // Reset odometry to the initial pose of the trajectory, run path following
      // command, then stop at the end.
      return Commands.sequence(
          new PrintCommand("hello again"),
          new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
          swerveControllerCommand,
          new InstantCommand(() -> swerve.drive(new Translation2d(0, 0), 0, false, false)));
    }
    return Commands.runOnce(() -> System.out.println("null"));
  }

  private double GetYaw() {
    if (trackedTarget != null) {
      return tagLayout.getTagPose(lockID).get().getRotation().getZ();
    }
    return 0.0;
  }

  private Translation2d GetTagTranslation() {

    if (lockID != 0) {

      var x = tagLayout.getTagPose(lockID).get().getX();
      var y = tagLayout.getTagPose(lockID).get().getY();

      return new Translation2d(x, y);
    }

    return new Translation2d();
  }
}
