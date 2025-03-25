// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.PathfindingV2;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.Vision;

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

  private static SendableChooser<Constants.AutoConstants.Sequence> autoChooser =
      new SendableChooser<>();

  private Swerve swerve;
  private Shooter shooter;
  private Climber climber;
  private AlgaeIntake algaeIntake;
  private Elevator elevator;
  // private CommandXboxController codriver;
  private Vision vision;

  ShuffleboardTab driverTab = Shuffleboard.getTab("driverTab");
  ShuffleboardTab techTab = Shuffleboard.getTab("techTab");
  ShuffleboardTab TestSubsystemTab = Shuffleboard.getTab("testSubsystemTab");
  ShuffleboardTab debugTab = Shuffleboard.getTab("debugTab");

  public ElasticSetup(
      Swerve swerve,
      Shooter shooter,
      Climber climber,
      AlgaeIntake algaeIntake,
      Elevator elevator,
      CommandXboxController codriver,
      Vision vision,
      PathfindingV2 pathfinder) {

    this.swerve = swerve;
    this.shooter = shooter;
    this.climber = climber;
    this.algaeIntake = algaeIntake;
    this.elevator = elevator;
    // this.codriver = codriver;
    this.vision = vision;

    autoChooser.setDefaultOption("null", null);
    autoChooser.addOption("three coral auto left", Constants.AutoConstants.Sequence.ThreeCoralLeft);
    autoChooser.addOption(
        "three coral auto right", Constants.AutoConstants.Sequence.ThreeCoralRight);
    autoChooser.addOption(
        "straight pipe coral then algae", Constants.AutoConstants.Sequence.OneCoralThenAlgae);
    autoChooser.addOption("Line (if no vision)", Constants.AutoConstants.Sequence.Line);
  }

  public void setUpDashboardComp() {

    // driver tab
    driverTab
        .addCamera("limelight", "limelight", "mjpg:http://10.33.60.11:1182/?action=stream")
        .withPosition(6, 0)
        .withSize(6, 4);
    driverTab
        .addCamera("driver cam", "driver cam", "mjpg:http://10.33.60.2:1181/?action=stream")
        .withPosition(0, 0)
        .withSize(6, 4);
    driverTab
        .addBoolean("has algae", () -> algaeIntake.sensorTriggered())
        .withPosition(0, 4)
        .withSize(1, 1);
    driverTab
        .addBoolean("climber acivated", () -> climber.isClimberActivated())
        .withPosition(1, 4)
        .withSize(1, 1);
    driverTab
        .addBoolean("climbed", () -> climber.SensorDetected())
        .withPosition(2, 4)
        .withSize(1, 1);
    driverTab.addBoolean("has coral", () -> shooter.isCoralIn()).withPosition(3, 4).withSize(1, 1);
    driverTab
        .addBoolean(
            "is in bounds for processor", () -> vision.isInBoundsForProcessor(swerve.getPose()))
        .withPosition(4, 4)
        .withSize(1, 1);
    driverTab
        .addInteger("reef side", () -> vision.getLockIDIndex())
        .withPosition(5, 4)
        .withSize(1, 1);

    driverTab
        .addString("elevator state", () -> elevator.getElevatorState().toString())
        .withPosition(6, 4);

    driverTab.addBoolean("peg beambreak", () -> algaeIntake.pegBeamBreak()).withPosition(7, 4);
    driverTab.addBoolean("target reached", () -> swerve.targetReached()).withPosition(8, 4);

    // technician tab
    //     - Autonomous mode (chooser)
    techTab.add(swerve.m_field2d).withPosition(6, 0).withSize(6, 4);
    // techTab
    //     .addBoolean("limelight 2 active", () -> vision.limelight2RightActive())
    //     .withPosition(0, 0)
    //     .withSize(1, 1);
    // techTab
    //     .addBoolean("limelight 3 active", () -> vision.limelight3Active())
    //     .withPosition(1, 0)
    //     .withSize(1, 1);

    techTab.add("auto chooser", autoChooser).withPosition(0, 1).withSize(3, 1);
  }

  public void setUpDashboardSubsystemTest() {

    // TestSubsystemTab.add(CommandScheduler.getInstance());
    // TestSubsystemTab.add("climber", setClimberMode());
    // TestSubsystemTab.add("elevator", setElevatorMode());
    // TestSubsystemTab.add("coral shooter", setCoralShooterMode());
    // TestSubsystemTab.add("algae intake", setAlgaeIntakeMode());
    // TestSubsystemTab.add("swerve", setSwerveMode());
    // TestSubsystemTab.add("dumper", setDumperMode());

    // shooter.setupTestBindings(new Trigger(isMode(TestModes.CORAL_SHOOTER)), codriver);

    // elevator.setupTestBindings(new Trigger(isMode(TestModes.ELEVATOR)), codriver);

    // climber.setupTestBindings(new Trigger(isMode(TestModes.CLIMBER)), codriver);

    // dumper.setupTestBindings(new Trigger(isMode(TestModes.DUMPER)), codriver);

    // algaeIntake.setupTestBindings(new Trigger(isMode(TestModes.ALGAE_INTAKE)), codriver);

    // swerve.setupTestBindings(new Trigger(isMode(TestModes.SWERVE)), codriver);
  }

  public void setUpDashboardDebug() {

    // selector
    debugTab.add("desiredPose x (left)", vision.getDesiredPoseLeft().getX());
    debugTab.add("desiredPose y (left)", vision.getDesiredPoseLeft().getY());
    debugTab.add("desiredPose x (right)", vision.getDesiredPoseRight().getX());
    debugTab.add("desiredPose y (right)", vision.getDesiredPoseRight().getY());
    debugTab.add("desiredPose x center", vision.getDesiredPoseAlgae(() -> swerve.getPose()).getX());
    debugTab.add("desiredPose y center", vision.getDesiredPoseAlgae(() -> swerve.getPose()).getY());
    debugTab.add("current pos x", swerve.getPose().getX());
    debugTab.add("current pos y", swerve.getPose().getY());
    debugTab.add("gyro rotation", swerve.getRotation2d().getDegrees());

    // algae intake

    debugTab.add("AlgaeEncoder", algaeIntake.getEncoderPosition());

    // climber

    debugTab.add("climber encoder", climber.getEncoderPosition());
    debugTab.add("gyro z", swerve.getGyroZ());

    // elevator

    debugTab.add("elevator pos", elevator.getElevatorPos());
    debugTab.add("elevator vel", elevator.elevatorVel());
    debugTab.add("elevator switch", elevator.isElevatorAtBottom());
    debugTab.add("elevator setpoint", elevator.getElevatorSetpoint());
  }

  // public static Command SelectedAuto() {
  // var selectedAuto = autoChooser.getSelected();
  // var autoCmd = Commands.none();
  // switch (selectedAuto) {
  //   case ThreeCoralLeft:
  //     autoCmd = pathfinder.ThreeCoralLeft();
  //     break;
  //   case ThreeCoralRight:
  //     autoCmd = pathfinder.ThreeCoralRight();
  //     break;
  //   case OneCoralThenAlgae:
  //     autoCmd = pathfinder.coralAndAlgae();
  //     break;
  //   case Line:
  //     autoCmd = pathfinder.straightLine();
  //   default:
  //     autoCmd = Commands.none();
  //     break;
  // }
  // return autoCmd;
  // }
  public static Constants.AutoConstants.Sequence SelectedAuto() {
    return autoChooser.getSelected();
  }
}
