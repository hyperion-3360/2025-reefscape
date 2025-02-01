// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class ShootCoralCmd extends SequentialCommandGroup {
  public ShootCoralCmd(Shooter m_shooter) {
    addRequirements(m_shooter);
    this.addCommands(
        Commands.runOnce(() -> System.out.println("I am running")),
        Commands.runOnce(() -> m_shooter.openBlocker()),
        new WaitCommand(0.3),
        Commands.run(() -> m_shooter.setShoot()).until(() -> m_shooter.isCoralIn()),
        new WaitCommand(0.3),
        Commands.runOnce(() -> m_shooter.stop()));

  }
}
