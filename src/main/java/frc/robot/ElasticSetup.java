// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Auto.PathfindingV2;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Dumper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.Selection;
import frc.robot.vision.Vision;
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

  public enum Auto {
    ThreeCoralLeft,
    ThreeCoralRight
  }

  private static SendableChooser<Auto> autoChooser = new SendableChooser<>();
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
  private Vision vision;
  private static PathfindingV2 pathfinder;
  
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
        CommandXboxController codriver,
        Vision vision, PathfindingV2 pathfinder) {
  
      this.swerve = swerve;
      this.shooter = shooter;
      this.climber = climber;
      this.selector = selector;
      this.algaeIntake = algaeIntake;
      this.elevator = elevator;
      this.dumper = dumper;
      this.codriver = codriver;
      this.vision = vision;
      ElasticSetup.pathfinder = pathfinder;

      autoChooser.setDefaultOption("null", null);
      autoChooser.addOption("three coral auto left", Auto.ThreeCoralLeft);
      autoChooser.addOption("three coral auto right", Auto.ThreeCoralRight);
      
    }
  
    public void setUpDashboardComp() {
  
      // driver tab
      driverTab
          .addCamera("limelight", "limelight", "mjpg:http://10.33.60.11:1182/?action=stream")
          .withPosition(6, 0)
          .withSize(6, 4);
      driverTab
          .addCamera("driverCam", "driverCam", "mjpg:http://10.33.60.2:1181")
          .withPosition(0, 0)
          .withSize(6, 4);
      driverTab
          .addBoolean("has algae", () -> algaeIntake.sensorTriggered())
          .withPosition(0, 3)
          .withSize(1, 1);
      driverTab
          .addBoolean("climber acivated", () -> climber.isClimberActivated())
          .withPosition(1, 3)
          .withSize(1, 1);
      driverTab
          .addBoolean("climbed", () -> climber.SensorDetected())
          .withPosition(2, 3)
          .withSize(1, 1);
      driverTab.addBoolean("has coral", () -> shooter.isCoralIn()).withPosition(3, 3).withSize(1, 1);
      driverTab
          .addBoolean("is in bounds for processor", () -> selector.isInBoundsForProcessor())
          .withPosition(4, 3)
          .withSize(1, 1);
      driverTab
          .addNumber("lock Tag id", () -> selector.getLockID())
          .withPosition(5, 3)
          .withSize(1, 1);
  
      // technician tab
      //     - Autonomous mode (chooser)
      techTab.add(swerve.m_field2d).withPosition(6, 0).withSize(6, 4);
      techTab.addBoolean("limelight 2 active", () -> vision.limelight2Active()).withPosition(0, 0).withSize(1, 1);
      techTab.addBoolean("limelight 3 active", () -> vision.limelight3Active()).withPosition(1, 0).withSize(1, 1);
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
  
    public static Command SelectedAuto() {
      var selectedAuto = autoChooser.getSelected();
      var autoCmd = Commands.none();
      switch (selectedAuto) {
        case ThreeCoralLeft:
        autoCmd = pathfinder.ThreeCoralLeft();
        break;
      case ThreeCoralRight:
      autoCmd = pathfinder.ThreeCoralRight();
        break;
      default:
      autoCmd = Commands.none();
        break;
      
    }
    return autoCmd;
  }
}
