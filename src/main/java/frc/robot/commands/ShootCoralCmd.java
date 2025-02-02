// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;

public class ShootCoralCmd extends SequentialCommandGroup {
  public ShootCoralCmd(Shooter m_shooter, LEDs m_leds) {
    addRequirements(m_shooter);
    addRequirements(m_leds);
    this.addCommands(
        m_leds
            .shootColor()
            .alongWith(
                Commands.runOnce(() -> m_shooter.openBlocker()),
                new WaitCommand(0.3),
                Commands.run(() -> m_shooter.setShoot()))
            .until(() -> m_shooter.isCoralIn()),
        new WaitCommand(0.3),
        Commands.runOnce(() -> m_shooter.stop()),
        Commands.runOnce(() -> m_shooter.closeBlocker()),
        m_leds.idleColor());
  }
}
