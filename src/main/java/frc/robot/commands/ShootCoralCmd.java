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
  public ShootCoralCmd(Shooter m_shooter, LEDs m_leds, Elevator m_elevator) {
    addRequirements(m_shooter);
    addRequirements(m_leds);
    addRequirements(m_elevator);

    // TODO fix shooter still turning when disabled
    this.addCommands(
        Commands.runOnce(() -> m_shooter.openBlocker()),
        new WaitCommand(0.3),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)),
        Commands.runOnce(() -> m_shooter.setShoot(getShootingSpeed(m_elevator.getTargetHeight())))
            .until(() -> !m_shooter.isCoralIn()),
        new WaitCommand(1.0),
        Commands.runOnce(() -> m_shooter.stop()),
        Commands.runOnce(() -> m_shooter.closeBlocker()),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.DONTPOUND))
            .unless(() -> !m_elevator.getTargetHeight().equals(desiredHeight.L4)),
        new WaitCommand(1.4)
            .unless(() -> !m_elevator.getTargetHeight().equals(desiredHeight.DONTPOUND)),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }

  private shootSpeed getShootingSpeed(desiredHeight height) {
    shootSpeed speed;
    switch (height) {
      case L1:
        speed = shootSpeed.L1;
        break;
      case L2:
        speed = shootSpeed.L2;
        break;
      case L3:
        speed = shootSpeed.L3;
        break;
      case L4:
        speed = shootSpeed.L4;
        break;
      default:
        speed = shootSpeed.STOP;
        break;
    }

    return speed;
  }
}
