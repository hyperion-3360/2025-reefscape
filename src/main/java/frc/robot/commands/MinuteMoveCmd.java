package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.swerve.Swerve;

public class MinuteMoveCmd extends SequentialCommandGroup {
  public static enum OffsetDir {
    FRONT,
    BACK,
    LEFT,
    RIGHT
  };

  private Swerve m_swerve;

  private Pose2d computeNewPose(double offset, OffsetDir dir) {
    // 0 deg in front of the robot
    var currentPose = m_swerve.getPose();
    double translationX = 0.0;
    double translationY = 0.0;
    switch (dir) {
      case FRONT:
        translationX += offset;
        break;
      case BACK:
        translationX -= offset;
        break;
      case LEFT:
        translationY += offset;
        break;
      case RIGHT:
        translationY -= offset;
        break;
    }
    var translation =
        currentPose
            .getTranslation()
            .plus(
                new Translation2d(translationX, translationY).rotateBy(currentPose.getRotation()));
    return new Pose2d(translation, currentPose.getRotation());
  }

  public MinuteMoveCmd(Swerve swerve, double waitTime, double offset, OffsetDir dir) {
    m_swerve = swerve;
    addRequirements(m_swerve);
    addCommands(
        new InstantCommand(() -> m_swerve.boostedConstraints()),
        new DeferredCommand(
            () -> new InstantCommand(() -> m_swerve.drivetoTarget(computeNewPose(offset, dir))),
            getRequirements()),
        new ParallelDeadlineGroup(
            new WaitCommand(waitTime), // Deadline command
            new WaitUntilCommand(() -> swerve.targetReached())),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()));
  }

  public MinuteMoveCmd(Swerve swerve, OffsetDir dir, AlgaeIntake algaeIntake) {
    m_swerve = swerve;
    addRequirements(m_swerve);
    addCommands(
        new DeferredCommand(
            () -> new InstantCommand(() -> m_swerve.drivetoTarget(computeNewPose(0.1, dir))),
            getRequirements()),
        new ParallelDeadlineGroup(
            new WaitUntilCommand(() -> algaeIntake.pegBeamBreak()), // Deadline command
            new WaitUntilCommand(() -> swerve.targetReached())),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()));
  }
}
