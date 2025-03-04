package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
    var originalX = m_swerve.getPose().getX();
    var originalY = m_swerve.getPose().getY();
    var originalRot = m_swerve.getPose().getRotation();

    switch (dir) {
      case FRONT:
        originalY += offset;
        break;
      case BACK:
        originalY -= offset;
        break;
      case LEFT:
        originalX -= offset;
        break;
      case RIGHT:
        originalX += offset;
        break;
    }
    return new Pose2d(new Translation2d(originalX, originalY).rotateBy(originalRot), originalRot);
  }

  public MinuteMoveCmd(Swerve swerve, double waitTime, double offset, OffsetDir dir) {
    m_swerve = swerve;
    addRequirements(m_swerve);
    addCommands(
        new InstantCommand(() -> m_swerve.drivetoTarget(computeNewPose(offset, dir))),
        new ParallelDeadlineGroup(
            new WaitCommand(waitTime), // Deadline command
            new WaitUntilCommand(() -> swerve.targetReached())),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()));
  }
}
