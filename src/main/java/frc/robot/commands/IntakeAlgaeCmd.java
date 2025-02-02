// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeIntake;

public class IntakeAlgaeCmd extends SequentialCommandGroup {
  AlgaeIntake m_algaeIntake;

  // start the intake rollers and wait until the algae is in the intake
  // then reduce the speed of the intake rollers and lift the intake to a preset position
  // then stop the intake rollers when desired elevation is reached

  public IntakeAlgaeCmd(AlgaeIntake m_algaeIntake, AlgaeIntake.elevation angle) {
    addRequirements(m_algaeIntake);
    addCommands(
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.FLOOR)),
        Commands.runOnce(
            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE), m_algaeIntake),
        new WaitCommand(1.5),
        new WaitUntilCommand(() -> m_algaeIntake.sensorTriggered()),
        Commands.runOnce(
            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORING), m_algaeIntake),
        Commands.run(() -> m_algaeIntake.setShootingAngle(angle), m_algaeIntake),
        new WaitUntilCommand(() -> m_algaeIntake.isAtAngle()),
        Commands.runOnce(
            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED), m_algaeIntake),
        Commands.runOnce(
            () -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.NET), m_algaeIntake));
  }
}
