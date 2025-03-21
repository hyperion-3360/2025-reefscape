// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MinuteMoveCmd.OffsetDir;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.PegDetect;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignPeg extends SequentialCommandGroup {

  private final double kMaxWaitAlignTime = 0.5;

  Swerve m_driveTrain;
  PegDetect m_pegDetection;

  /** Creates a new VisionAlignPeg. */
  public AlignPeg(Swerve driveTrain, PegDetect pegDetection) {
    m_driveTrain = driveTrain;
    m_pegDetection = pegDetection;

    addRequirements(m_driveTrain);

    addCommands(
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
  }
}
