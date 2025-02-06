// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shootSpeed;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

public class ShootCoralCmd extends SequentialCommandGroup {
  public ShootCoralCmd(Shooter m_shooter, LEDs m_leds, Elevator m_elevator, desiredHeight height) {
    addRequirements(m_shooter);
    addRequirements(m_leds);
    addRequirements(m_elevator);

    this.addCommands(
        Commands.runOnce(() -> m_shooter.openBlocker()),
        new WaitCommand(0.3),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)),
        Commands.runOnce(() -> m_shooter.setShoot(getShootingSpeed(height)))
            .until(() -> !m_shooter.isCoralIn()),
        new WaitCommand(1.2),
        Commands.runOnce(() -> m_shooter.stop()),
        Commands.runOnce(() -> m_shooter.closeBlocker()),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }

  private shootSpeed getShootingSpeed(desiredHeight height) {
    shootSpeed speed =
        switch (height) {
          case L1 -> {
            yield shootSpeed.L1;
          }
          case L2 -> {
            yield shootSpeed.L2;
          }
          case L3 -> {
            yield shootSpeed.L3;
          }
          case L4 -> {
            yield shootSpeed.L4;
          }
          default -> {
            yield shootSpeed.STOP;
          }
        };
    return speed;
  }
}
