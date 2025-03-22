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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.PegDetect;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignPeg extends SequentialCommandGroup {

  private final double kMaxWaitAlignTime = 0.5;

  Swerve m_driveTrain;
  PegDetect m_pegDetection;
  Elevator m_elevator;

  /** Creates a new VisionAlignPeg. */
  public AlignPeg(
      Swerve driveTrain, Elevator elevator, PegDetect pegDetection, Pose2d desiredPose) {
    m_driveTrain = driveTrain;
    m_pegDetection = pegDetection;
    m_elevator = elevator;

    addRequirements(m_driveTrain);
    addRequirements(m_elevator);

    addCommands(
        // new PrintCommand("this has started Yayyyyyyyyyyyyyyyyyyyyyyyyy"),
        new ParallelDeadlineGroup(
            new WaitCommand(1.5),
            Commands.runOnce(() -> m_driveTrain.drivetoTarget(desiredPose)),
            new WaitUntilCommand(() -> m_driveTrain.targetReached())),
        new InstantCommand(() -> m_driveTrain.disableDriveToTarget()),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.L4)),
        new WaitCommand(1.5),
        // Commands.runOnce(() -> m_elevator.AutoElevate()),
        // this is one command
        new ConditionalCommand(
            new DeferredCommand(
                () ->
                    new MinuteMoveCmd(
                        m_driveTrain,
                        kMaxWaitAlignTime,
                        Math.abs(m_pegDetection.getOffset()),
                        (m_pegDetection.getOffset() < 0) ? OffsetDir.LEFT : OffsetDir.RIGHT),
                getRequirements()),
            new PrintCommand("Can't locate peg!!! "),
            () -> m_pegDetection.processImage()));
    // ends command for peg correction
  }
}
