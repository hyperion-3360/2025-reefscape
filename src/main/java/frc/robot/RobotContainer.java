// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.Joysticks;
import frc.robot.commands.ElevateCmd;
import frc.robot.commands.IntakeAlgaeCmd;
import frc.robot.commands.IntakeCoralCmd;
import frc.robot.commands.LowerElevatorCmd;
import frc.robot.commands.NetAlgaeShootCmd;
import frc.robot.commands.ShootAlgaeCmd;
import frc.robot.commands.ShootCoralCmd;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.elevation;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Dumper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
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
  private static SendableChooser<Command> m_climberCommand = new SendableChooser<>();

  // Joystick axis declarations
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Slew Rate Limiters to limit acceleration of joystick inputs
  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

  private final SlewRateLimiter climberLimiter = new SlewRateLimiter(3);

  private final IntakeAlgaeCmd intakeAlgaeFloor =
      new IntakeAlgaeCmd(m_algaeIntake, elevation.FLOOR, m_leds, m_elevator, desiredHeight.LOW);
  private final IntakeAlgaeCmd intakeAlgaeL2 =
      new IntakeAlgaeCmd(m_algaeIntake, elevation.FLOOR, m_leds, m_elevator, desiredHeight.ALGAEL2);
  private final IntakeAlgaeCmd intakeAlgaeL3 =
      new IntakeAlgaeCmd(m_algaeIntake, elevation.FLOOR, m_leds, m_elevator, desiredHeight.ALGAEL3);
  private final ShootAlgaeCmd shootAlgae = new ShootAlgaeCmd(m_algaeIntake, m_elevator, m_leds);
  private final NetAlgaeShootCmd shootAlgaeNet =
      new NetAlgaeShootCmd(m_algaeIntake, m_leds, m_elevator);

  private final ShootCoralCmd shootCoral = new ShootCoralCmd(m_shooter, m_leds, m_elevator);
  private final IntakeCoralCmd intakeCoral = new IntakeCoralCmd(m_shooter, m_elevator, m_leds);

  private final ElevateCmd elevateL1 =
      new ElevateCmd(m_elevator, m_shooter, m_algaeIntake, m_leds, desiredHeight.L1);
  private final ElevateCmd elevateL2 =
      new ElevateCmd(m_elevator, m_shooter, m_algaeIntake, m_leds, desiredHeight.L2);
  private final ElevateCmd elevateL3 =
      new ElevateCmd(m_elevator, m_shooter, m_algaeIntake, m_leds, desiredHeight.L3);
  private final ElevateCmd elevateL4 =
      new ElevateCmd(m_elevator, m_shooter, m_algaeIntake, m_leds, desiredHeight.L4);
  private final LowerElevatorCmd elevateLOW =
      new LowerElevatorCmd(m_elevator, m_leds, m_shooter, m_algaeIntake);

  public enum TestModes {
    NONE,
    ELEVATOR,
    CLIMBER,
    ALGAE_INTAKE,
    SWERVE,
    DUMPER,
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

  Command setDumperMode() {
    return Commands.runOnce(() -> m_testMode = TestModes.DUMPER);
  }

  BooleanSupplier isMode(TestModes mode) {
    return () -> m_testMode == mode;
  }

  public RobotContainer() {

    // Auto.initAutoWidget();
    // Pathfinding.configurePathfinder(m_shooter, m_swerve, m_elevator, m_algaeIntake, m_dumper);

    m_swerve.resetModulesToAbsolute();
    SmartDashboard.putData(CommandScheduler.getInstance());

    SmartDashboard.putData("Climber", setClimberMode());
    SmartDashboard.putData("Elevator", setElevatorMode());
    SmartDashboard.putData("Coral Shooter", setCoralShooterMode());
    SmartDashboard.putData("Algae Intake", setAlgaeIntakeMode());
    SmartDashboard.putData("Swerve", setSwerveMode());
    SmartDashboard.putData("Dumper", setDumperMode());
    m_climberCommand.addOption(
        "Shallow",
        m_climber.shallowClimb(
            () ->
                Joysticks.conditionJoystick(
                    () -> m_coDriverController.getLeftY(),
                    climberLimiter,
                    Constants.stickDeadband,
                    true)));
    m_climberCommand.setDefaultOption(
        "Deep",
        m_climber.deepClimb(
            () ->
                Joysticks.conditionJoystick(
                    () -> m_coDriverController.getLeftY(),
                    climberLimiter,
                    Constants.stickDeadband,
                    true)));

    SmartDashboard.putData("Climber Mode", m_climberCommand);

    PathPlannerLogging.logCurrentPose(m_swerve.getPose());

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
            () -> m_elevator.getEncoderPos());

    m_swerve.setDefaultCommand(teleopCmd);
  }

  public void configureBindingsTest() {

    m_shooter.setupTestBindings(new Trigger(isMode(TestModes.CORAL_SHOOTER)), m_coDriverController);

    m_elevator.setupTestBindings(new Trigger(isMode(TestModes.ELEVATOR)), m_coDriverController);

    m_climber.setupTestBindings(new Trigger(isMode(TestModes.CLIMBER)), m_coDriverController);

    m_dumper.setupTestBindings(new Trigger(isMode(TestModes.DUMPER)), m_coDriverController);

    m_algaeIntake.setupTestBindings(
        new Trigger(isMode(TestModes.ALGAE_INTAKE)), m_coDriverController);

    m_swerve.setupTestBindings(new Trigger(isMode(TestModes.SWERVE)), m_coDriverController);
  }

  public void configureBindingsTeleop() {

    m_driverController
        .x()
        .toggleOnTrue(intakeAlgaeFloor)
        .toggleOnFalse(intakeAlgaeL2.NoAlgaeCmd(m_elevator, m_algaeIntake, m_leds));

    m_driverController.a().onTrue(intakeCoral);
    m_driverController.b().onTrue(shootAlgae);

    m_climber.setDefaultCommand(m_climberCommand.getSelected());

    m_coDriverController.a().onTrue(shootCoral);
    m_driverController.y().onTrue(shootAlgaeNet);

    m_coDriverController
        .y()
        .whileTrue(intakeAlgaeL2)
        .onFalse(intakeAlgaeL2.NoAlgaeCmd(m_elevator, m_algaeIntake, m_leds));
    m_coDriverController
        .x()
        .whileTrue(intakeAlgaeL3)
        .onFalse(intakeAlgaeL3.NoAlgaeCmd(m_elevator, m_algaeIntake, m_leds));

    m_coDriverController.povUp().onTrue(elevateL4);
    m_coDriverController.povDown().onTrue(elevateL1);
    m_coDriverController.povLeft().onTrue(elevateL3);
    m_coDriverController.povRight().onTrue(elevateL2);
    m_coDriverController.b().onTrue(elevateLOW);

    m_driverController
        .povUp()
        .onTrue(
            Commands.runOnce(() -> m_swerve.drivetoTarget(m_selector.getDesiredposeAlgae()))
            //          .andThen(new WaitUntilCommand(m_swerve::targetReached).andThen(() ->
            // m_leds.SetPattern(LEDs.Pattern.READY)))
            //         .raceWith(new WaitUntilCommand(m_swerve::targetDriveDisabled).andThen(() ->
            // m_leds.SetPattern(LEDs.Pattern.READY)))
            )
        .onFalse(Commands.runOnce(() -> m_swerve.disableDriveToTarget()));

    m_driverController
        .leftBumper()
        .onTrue(
            Commands.runOnce(() -> m_swerve.drivetoTarget(m_selector.getDesiredposeLeft()))
            //          .andThen(new WaitUntilCommand(m_swerve::targetReached).andThen(() ->
            // m_leds.SetPattern(LEDs.Pattern.READY)))
            //         .raceWith(new WaitUntilCommand(m_swerve::targetDriveDisabled).andThen(() ->
            // m_leds.SetPattern(LEDs.Pattern.READY)))
            )
        .onFalse(Commands.runOnce(() -> m_swerve.disableDriveToTarget()));

    m_driverController
        .rightBumper()
        .onTrue(
            Commands.runOnce(() -> m_swerve.drivetoTarget(m_selector.getDesiredposeRight()))
            //          .andThen(new WaitUntilCommand(m_swerve::targetReached).andThen(() ->
            // m_leds.SetPattern(LEDs.Pattern.READY)))
            //         .raceWith(new WaitUntilCommand(m_swerve::targetDriveDisabled).andThen(() ->
            // m_leds.SetPattern(LEDs.Pattern.READY)))
            )
        .onFalse(Commands.runOnce(() -> m_swerve.disableDriveToTarget()));
  }

  public Command getAutonomousCommand() {
    m_swerve.setPose(new Pose2d(2, 6, Rotation2d.fromDegrees(180)));
    return Pathfinding.doPathfinding();
  }
}
