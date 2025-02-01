// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class IntakeCoralCmd extends SequentialCommandGroup {
  public IntakeCoralCmd(Shooter m_shooter) {
    this.addCommands(
        Commands.runOnce(() -> m_shooter.closeBlocker()),
        new WaitCommand(0.3),
        Commands.run(() -> m_shooter.setShoot()).until(() -> !m_shooter.isCoralIn()),
        new WaitCommand(0.8),
        Commands.runOnce(() -> m_shooter.stop()));
  }

  public Command cancelCommand() {
    return Commands.runOnce(() -> this.cancel());
  }
}
