// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.MinuteMoveCmd.OffsetDir;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shootSpeed;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;

/** a special command for L1 : we need a special sequence for L1 */
public class ShootCoralL1Cmd extends SequentialCommandGroup {
  private double sideTrackTime = 0.46;

  public ShootCoralL1Cmd(Shooter m_shooter, LEDs m_leds, Elevator m_elevator, Swerve m_swerve) {
    addRequirements(m_shooter);
    addRequirements(m_leds);
    addRequirements(m_elevator);

    MinuteMoveCmd sideTrack = new MinuteMoveCmd(m_swerve, sideTrackTime, 0.5, OffsetDir.RIGHT);

    this.addCommands(
        Commands.runOnce(() -> m_shooter.openBlocker()),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)),
        Commands.runOnce(() -> m_shooter.setShoot(shootSpeed.L1)),
        // once we don't detect any coral move it a bit further to make it loose
        new WaitUntilCommand(() -> !m_shooter.isCoralIn()),
        Commands.runOnce(() -> m_shooter.stop()),
        sideTrack,
        new WaitCommand(0.1),
        Commands.runOnce(() -> m_shooter.setShoot(shootSpeed.L1)),
        new WaitCommand(sideTrackTime - 0.1),
        Commands.runOnce(() -> m_shooter.stop()),
        Commands.runOnce(() -> m_shooter.closeBlocker()),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }
}
