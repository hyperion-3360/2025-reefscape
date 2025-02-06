// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.Joysticks;
import frc.robot.Auto.Auto;
import frc.robot.Auto.Pathfinding;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Dumper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.Patterns;
import frc.robot.subsystems.swerve.CTREConfigs;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.Selection;
import frc.robot.vision.Vision;
import java.util.function.BooleanSupplier;

public class RobotContainer {

  // controller declarations
  public static final CommandXboxController m_driverController = new CommandXboxController(0);
  public static final CommandXboxController m_coDriverController = new CommandXboxController(1);

  // subsystem declarations
  public static final Shooter m_shooter = new Shooter();
  public static final Vision m_vision = new Vision();
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  public static final Swerve m_swerve = new Swerve(m_vision);
  public static final AlgaeIntake m_algaeIntake = new AlgaeIntake();
  public static final Climber m_climber = new Climber();
  public static final Elevator m_elevator = new Elevator();
  public static final LEDs m_leds = new LEDs();
  public static final Patterns m_patterns = new Patterns();
  public static final Dumper m_dumper = new Dumper();
  public static final Selection m_selector = new Selection(m_swerve);

  // Joystick axis declarations
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Slew Rate Limiters to limit acceleration of joystick inputs
  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

  public enum TestModes {
    NONE,
    ELEVATOR,
    CLIMBER,
    ALGAE_INTAKE,
    SWERVE,
    CORAL_SHOOTER
  }

  private TestModes m_testMode = TestModes.NONE;

  Command setElevatorMode() {
    return Commands.runOnce(() -> m_testMode = TestModes.ELEVATOR);
  }

  Command setClimberMode() {
    return Commands.runOnce(() -> m_testMode = TestModes.CLIMBER);
  }

  Command setCoralShooterMode() {
    return Commands.runOnce(() -> m_testMode = TestModes.CORAL_SHOOTER);
  }

  Command setAlgaeIntakeMode() {
    return Commands.runOnce(() -> m_testMode = TestModes.ALGAE_INTAKE);
  }

  Command setSwerveMode() {
    return Commands.runOnce(() -> m_testMode = TestModes.SWERVE);
  }

  BooleanSupplier isMode(TestModes mode) {
    return () -> m_testMode == mode;
  }

  public RobotContainer() {

    Auto.initAutoWidget();
    Pathfinding.configurePathfinder(m_shooter, m_swerve, m_elevator, m_algaeIntake, m_dumper);

    m_swerve.resetModulesToAbsolute();
    SmartDashboard.putData(CommandScheduler.getInstance());

    SmartDashboard.putData("Climber", setClimberMode());
    SmartDashboard.putData("Elevator", setElevatorMode());
    SmartDashboard.putData("Coral Shooter", setCoralShooterMode());
    SmartDashboard.putData("Algae Intake", setAlgaeIntakeMode());
    SmartDashboard.putData("Swerve", setSwerveMode());

    var teleopCmd =
        new TeleopSwerve(
            m_swerve,
            () ->
                Joysticks.conditionJoystick(
                    () -> m_driverController.getRawAxis(translationAxis),
                    translationLimiter,
                    Constants.stickDeadband,
                    true),
            () ->
                Joysticks.conditionJoystick(
                    () -> m_driverController.getRawAxis(strafeAxis),
                    strafeLimiter,
                    Constants.stickDeadband,
                    true),
            () ->
                Joysticks.conditionJoystick(
                    () -> m_driverController.getRawAxis(rotationAxis),
                    rotationLimiter,
                    Constants.stickDeadband,
                    true),
            () -> true,
            m_elevator.getEncoderPos());

    m_swerve.setDefaultCommand(teleopCmd);
  }

  public void configureBindingsTest() {

    m_shooter.setupTestBindings(new Trigger(isMode(TestModes.CORAL_SHOOTER)), m_coDriverController);

    m_elevator.setupTestBindings(new Trigger(isMode(TestModes.ELEVATOR)), m_coDriverController);

    m_climber.setupTestBindings(new Trigger(isMode(TestModes.CLIMBER)), m_coDriverController);

    m_algaeIntake.setupTestBindings(
        new Trigger(isMode(TestModes.ALGAE_INTAKE)), m_coDriverController);
  }

  public void configureBindingsTeleop() {
    m_coDriverController.a().onTrue(m_elevator.Elevate(Elevator.desiredHeight.LOW));

    m_coDriverController.x().onTrue(m_elevator.Elevate(Elevator.desiredHeight.L4));

    m_coDriverController.y().onTrue(m_elevator.Elevate(Elevator.desiredHeight.L1));

    m_coDriverController.b().onTrue(m_elevator.Elevate(Elevator.desiredHeight.L2));
  }

  public Command getAutonomousCommand() {
    return Pathfinding.fullControl();
  }
}
