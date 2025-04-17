// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.MinuteMoveCmd.OffsetDir;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.PegDetect;
import frc.robot.vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignPeg extends SequentialCommandGroup {

  private final double kMaxWaitAlignTime = 1;

  Swerve m_driveTrain;
  PegDetect m_pegDetection;
  Elevator m_elevator;
  Shooter m_shooter;
  Direction m_direction;
  Vision m_vision;

  public enum Direction {
    left,
    right
  }

  /** Creates a new VisionAlignPeg. */
  public AlignPeg(
      Swerve driveTrain,
      Elevator elevator,
      Shooter shooter,
      AlgaeIntake beambreak,
      PegDetect pegDetection,
      Vision vision,
      Direction direction) {
    m_driveTrain = driveTrain;
    m_pegDetection = pegDetection;
    m_elevator = elevator;
    m_shooter = shooter;
    m_direction = direction;
    m_vision = vision;

    addRequirements(m_driveTrain);
    addRequirements(m_elevator);
    addRequirements(m_shooter);

    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(2.5),
                    Commands.runOnce(() -> m_driveTrain.drivetoTarget(direction())),
                    Commands.runOnce(() -> m_elevator.AutoElevate()),
                    Commands.runOnce(() -> m_shooter.openBlocker()),
                    new WaitUntilCommand(() -> m_driveTrain.targetReached())),
                new InstantCommand(() -> m_driveTrain.disableDriveToTarget()),
                // Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.L4)),
                // new WaitUntilCommand(() -> m_driveTrain.targetReached()),
                // new WaitCommand(1),
                // this is one command
                new WaitUntilCommand(() -> m_elevator.isAtGoal())),
            new PrintCommand("hehe"),
            () -> m_vision.getLockID() != 0));

    // ends command for peg correction
  }

  private Pose2d direction() {
    var desiredPose = new Pose2d();

    if (m_direction == Direction.left) {
      desiredPose = m_vision.getDesiredPoseLeft();
    } else if (m_direction == Direction.right) {
      desiredPose = m_vision.getDesiredPoseRight();
    } else {
      desiredPose = Pose2d.kZero;
    }

    return desiredPose;
  }

  public AlignPeg(
      Swerve driveTrain,
      Elevator elevator,
      Shooter shooter,
      AlgaeIntake beambreak,
      PegDetect pegDetection,
      Vision vision,
      Pose2d pose,
      desiredHeight elevatorHeight) {
    m_driveTrain = driveTrain;
    m_pegDetection = pegDetection;
    m_elevator = elevator;
    m_shooter = shooter;
    m_vision = vision;

    addRequirements(m_driveTrain);
    addRequirements(m_elevator);
    addRequirements(m_shooter);

    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(2.5),
                    Commands.runOnce(() -> m_driveTrain.drivetoTarget(pose)),
                    Commands.runOnce(() -> m_elevator.SetHeight(elevatorHeight)),
                    Commands.runOnce(() -> m_shooter.openBlocker()),
                    new WaitUntilCommand(() -> m_driveTrain.targetReached())),
                new InstantCommand(() -> m_driveTrain.disableDriveToTarget()),
                // Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.L4)),
                // new WaitUntilCommand(() -> m_driveTrain.targetReached()),
                // new WaitCommand(1),
                // this is one command
                new WaitUntilCommand(m_elevator::isAtGoal),
                new ConditionalCommand(
                    new DeferredCommand(
                        () ->
                            new MinuteMoveCmd(
                                m_driveTrain,
                                kMaxWaitAlignTime,
                                Math.abs(m_pegDetection.getOffset()),
                                (m_pegDetection.getOffset() < 0)
                                    ? OffsetDir.LEFT
                                    : OffsetDir.RIGHT),
                        getRequirements()),
                    new PrintCommand("Can't locate peg!!! "),
                    () -> {
                      return m_pegDetection.processImage();
                      //                          && m_elevator.getTargetHeight() !=
                      // desiredHeight.LOW;
                    })),
            new PrintCommand("hehe"),
            () -> m_vision.getLockID() != 0));

    // ends command for peg correction
  }
}
