// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Dumper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.Selection;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class ElasticSetup {

  private enum TestModes {
    NONE,
    ELEVATOR,
    CLIMBER,
    ALGAE_INTAKE,
    SWERVE,
    DUMPER,
    CORAL_SHOOTER
  }

  private TestModes m_testMode = TestModes.NONE;

  private Command setElevatorMode() {
    return Commands.runOnce(() -> m_testMode = TestModes.ELEVATOR);
  }

  private Command setClimberMode() {
    return Commands.runOnce(() -> m_testMode = TestModes.CLIMBER);
  }

  private Command setCoralShooterMode() {
    return Commands.runOnce(() -> m_testMode = TestModes.CORAL_SHOOTER);
  }

  private Command setAlgaeIntakeMode() {
    return Commands.runOnce(() -> m_testMode = TestModes.ALGAE_INTAKE);
  }

  private Command setSwerveMode() {
    return Commands.runOnce(() -> m_testMode = TestModes.SWERVE);
  }

  private Command setDumperMode() {
    return Commands.runOnce(() -> m_testMode = TestModes.DUMPER);
  }

  private BooleanSupplier isMode(TestModes mode) {
    return () -> m_testMode == mode;
  }

  private Swerve swerve;
  private Shooter shooter;
  private Climber climber;
  private Selection selector;
  private AlgaeIntake algaeIntake;
  private Elevator elevator;
  private Dumper dumper;
  private CommandXboxController codriver;

  ShuffleboardTab driverTab = Shuffleboard.getTab("driverTab");
  ShuffleboardTab techTab = Shuffleboard.getTab("techTab");
  ShuffleboardTab TestSubsystemTab = Shuffleboard.getTab("testSubsystemTab");
  ShuffleboardTab debugTab = Shuffleboard.getTab("debugTab");

  public ElasticSetup(
      Swerve swerve,
      Shooter shooter,
      Climber climber,
      Selection selector,
      AlgaeIntake algaeIntake,
      Elevator elevator,
      Dumper dumper,
      CommandXboxController codriver) {

    this.swerve = swerve;
    this.shooter = shooter;
    this.climber = climber;
    this.selector = selector;
    this.algaeIntake = algaeIntake;
    this.elevator = elevator;
    this.dumper = dumper;
    this.codriver = codriver;
  }

  public void setUpDashboardComp() {

    // driver tab
    driverTab.add(swerve.m_field2d).withPosition(5, 0).withSize(6, 4);
    driverTab
        .addCamera("limelight", "limelight", "mjpg:http://10.33.60.11:1182/?action=stream")
        .withPosition(3, 0)
        .withSize(3, 3);
    driverTab
        .addCamera("driverCam", "driverCam", "mjpg:http://10.33.60.2:1181/?action=stream")
        .withPosition(0, 0)
        .withSize(3, 3);
    driverTab.addBoolean("has algae", () -> algaeIntake.sensorTriggered());
    driverTab.addBoolean("climber acivated", () -> climber.isClimberActivated());
    driverTab.addBoolean("climbed", () -> climber.SensorDetected());
    driverTab.addBoolean("has coral", () -> shooter.isCoralIn());
    driverTab.addBoolean("is in bounds for processor", () -> selector.isInBoundsForProcessor());
    driverTab.addNumber("lock Tag id", () -> selector.getLockID());

    // technician tab

  }

  public void setUpDashboardSubsystemTest() {

    TestSubsystemTab.add(CommandScheduler.getInstance());
    TestSubsystemTab.add("climber", setClimberMode());
    TestSubsystemTab.add("elevator", setElevatorMode());
    TestSubsystemTab.add("coral shooter", setCoralShooterMode());
    TestSubsystemTab.add("algae intake", setAlgaeIntakeMode());
    TestSubsystemTab.add("swerve", setSwerveMode());
    TestSubsystemTab.add("dumper", setDumperMode());

    shooter.setupTestBindings(new Trigger(isMode(TestModes.CORAL_SHOOTER)), codriver);

    elevator.setupTestBindings(new Trigger(isMode(TestModes.ELEVATOR)), codriver);

    climber.setupTestBindings(new Trigger(isMode(TestModes.CLIMBER)), codriver);

    dumper.setupTestBindings(new Trigger(isMode(TestModes.DUMPER)), codriver);

    algaeIntake.setupTestBindings(new Trigger(isMode(TestModes.ALGAE_INTAKE)), codriver);

    swerve.setupTestBindings(new Trigger(isMode(TestModes.SWERVE)), codriver);
  }

  public void setUpDashboardDebug() {

    // selector
    debugTab.add("desiredPose x (left)", selector.getDesiredposeLeft().getX());
    debugTab.add("desiredPose y (left)", selector.getDesiredposeLeft().getY());
    debugTab.add("desiredPose x (right)", selector.getDesiredposeRight().getX());
    debugTab.add("desiredPose y (right)", selector.getDesiredposeRight().getY());
    debugTab.add("desiredPose x center", selector.getDesiredposeAlgae().getX());
    debugTab.add("desiredPose y center", selector.getDesiredposeAlgae().getY());
    debugTab.add("current pos x", swerve.getPose().getX());
    debugTab.add("current pos y", swerve.getPose().getY());
    debugTab.add("gyro rotation", swerve.getRotation2d().getDegrees());

    // algae intake

    debugTab.add("AlgaeEncoder", algaeIntake.getEncoderPosition());

    // climber

    debugTab.add("climber encoder", climber.getEncoderPosition());

    // elevator

    debugTab.add("elevator pos", elevator.getElevatorPos());
    debugTab.add("elevator vel", elevator.elevatorVel());
    debugTab.add("elevator switch", elevator.isElevatorAtBottom());
    debugTab.add("elevator setpoint", elevator.getElevatorSetpoint());
  }
}
