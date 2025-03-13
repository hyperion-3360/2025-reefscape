// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.PathfindingV2;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.elevation;
import frc.robot.subsystems.AlgaeIntake.shooting;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.Selection;

public class IntakeAlgaeCmd extends SequentialCommandGroup {
  AlgaeIntake m_algaeIntake;
  desiredHeight m_height;

  // start the intake rollers and wait until the algae is in the intake
  // then reduce the speed of the intake rollers and lift the intake to a preset position
  // then stop the intake rollers when desired elevation is reached

  public IntakeAlgaeCmd(
      AlgaeIntake m_algaeIntake,
      LEDs m_leds,
      Elevator m_elevator,
      desiredHeight height,
      CommandXboxController m_controller) {
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    addRequirements(m_elevator);
    addCommands(
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_elevator.SetHeight(height)),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.FLOOR)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.INTAKE)),
        Commands.runOnce(
            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE), m_algaeIntake),
        new WaitUntilCommand(
            () -> m_algaeIntake.sensorTriggered() || m_controller.x().getAsBoolean()),
        new WaitCommand(0.5),
        Commands.runOnce(
                () -> {
                  m_algaeIntake.setShootingSpeed(shooting.STORED);
                  m_algaeIntake.setShootingAngle(elevation.STORED);
                  m_leds.SetPattern(Pattern.IDLE);
                })
            .onlyIf(() -> m_controller.x().getAsBoolean()),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.READY)),
        Commands.runOnce(
            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORING), m_algaeIntake),
        Commands.runOnce(
            () -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.NET), m_algaeIntake),
        new WaitUntilCommand(() -> m_algaeIntake.isAtAngle(0.3)),
        Commands.runOnce(
            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORING), m_algaeIntake));
  }
/**
 * constructor to construct a algae intake command for the reef
 * @param m_algaeIntake
 * @param m_leds
 * @param m_elevator
 * @param height
 */
  public IntakeAlgaeCmd(
      AlgaeIntake m_algaeIntake, LEDs m_leds, Elevator m_elevator, desiredHeight height, Swerve m_swerve, Selection m_selector) {
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    addRequirements(m_elevator);
    m_height = height;
    addCommands(
        Commands.runOnce(() -> m_swerve.drivetoTarget(m_selector.getDesiredposeAlgae())),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_elevator.SetHeight(height)),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.FLOOR)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.INTAKE)),
        Commands.runOnce(
            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE), m_algaeIntake),
        new WaitCommand(1.0),
        new WaitUntilCommand(() -> m_algaeIntake.sensorTriggered()),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.READY)));
  }

  public Command NoAlgaeCmd(Elevator m_elevator, AlgaeIntake m_algaeIntake, LEDs m_leds) {
    addRequirements(m_elevator);
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    return Commands.sequence(
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORING)),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.STORED)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }
}
