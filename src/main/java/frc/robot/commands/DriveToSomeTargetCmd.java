package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.Supplier;

public class DriveToSomeTargetCmd extends SequentialCommandGroup {
  public DriveToSomeTargetCmd(Supplier<Pose2d> pose, Swerve m_swerve) {
    addCommands(
        new InstantCommand(() -> m_swerve.drivetoTarget(pose.get())),
        new WaitUntilCommand(() -> m_swerve.targetReached()),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()));
  }
}
