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
import frc.robot.subsystems.leds.LEDs;

public class IntakeCoralCmd extends SequentialCommandGroup {
  public IntakeCoralCmd(Shooter m_shooter, Elevator m_elevator, LEDs m_leds) {
    addRequirements(m_shooter);
    addRequirements(m_leds);
    addRequirements(m_elevator);
    this.addCommands(
        m_leds
            .elevatingColor()
            .alongWith(
                Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.FEEDER)),
                Commands.runOnce(() -> m_shooter.closeBlocker())),
        m_leds
            .intakeColors()
            .alongWith(Commands.run(() -> m_shooter.setIntake()))
            .until(() -> m_shooter.isCoralIn())
            .andThen(new WaitCommand(0.3), Commands.runOnce(() -> m_shooter.stop()))
            .andThen(m_leds.readyColor()));
  }
}
