// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Dumper;

public class AutoDump extends SequentialCommandGroup {

  public AutoDump(Dumper m_dumper) {
    addRequirements(m_dumper);
    addCommands(
        Commands.print("soNe-yOuh :)"),
        Commands.runOnce(
            () -> {
              m_dumper.dumpLeft();
              m_dumper.dumpRight();
            }),
        Commands.print("hello"));
  }

  public Command cancelDumper(Dumper m_dumper) {
    addRequirements(m_dumper);
    return Commands.runOnce(
        () -> {
          m_dumper.closeDump();
        });
  }
}
