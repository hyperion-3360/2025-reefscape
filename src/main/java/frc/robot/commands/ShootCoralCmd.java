// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;

public class ShootCoralCmd extends SequentialCommandGroup {
  public ShootCoralCmd(Shooter m_shooter) {
    this.addCommands(
    m_shooter.openBlocker(),
    new WaitCommand(0.3),
    Commands.runOnce(() -> m_shooter.setShoot()),
    new WaitUntilCommand(() -> !m_shooter.isCoralIn()),
    Commands.runOnce(() -> m_shooter.stop())
    );
  }

  public Command cancelCommand() {
    return Commands.runOnce(() -> this.cancel());
  }
}
