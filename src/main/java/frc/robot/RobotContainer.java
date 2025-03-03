// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.Joysticks;
import frc.robot.Auto.PathfindingV2;
import frc.robot.commands.AutoCmd.AutoCancel;
import frc.robot.commands.AutoCmd.AutoDump;
import frc.robot.commands.AutoCmd.AutoFeast;
import frc.robot.commands.AutoCmd.AutoFeeder;
import frc.robot.commands.DeepClimbCmd;
import frc.robot.commands.ElevateCmd;
import frc.robot.commands.IntakeAlgaeCmd;
import frc.robot.commands.IntakeCoralCmd;
import frc.robot.commands.LowerElevatorCmd;
import frc.robot.commands.NetAlgaeShootCmd;
import frc.robot.commands.ReadyClimbCmd;
import frc.robot.commands.ShootAlgaeCmd;
import frc.robot.commands.ShootCoralCmd;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AlgaeIntake;
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

public class RobotContainer {

  // controller declarations
  public static final CommandXboxController m_driverController = new CommandXboxController(0);
  public static final CommandXboxController m_coDriverController = new CommandXboxController(1);

  // subsystem declarations
  public static final Shooter m_shooter = new Shooter();
  public static final Vision m_vision = new Vision();
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  public static final AlgaeIntake m_algaeIntake = new AlgaeIntake();
  public static final Climber m_climber = new Climber();
  public static final Elevator m_elevator = new Elevator();
  public static final Swerve m_swerve = new Swerve(m_vision, m_elevator);
  public static final LEDs m_leds = new LEDs();
  public static final Patterns m_patterns = new Patterns();
  public static final Dumper m_dumper = new Dumper();
  public static final Selection m_selector = new Selection(m_swerve);
  public static final PathfindingV2 m_pathfinding =
      new PathfindingV2(m_shooter, m_elevator, m_leds, m_swerve);

  public ElasticSetup setup =
      new ElasticSetup(
          m_swerve,
          m_shooter,
          m_climber,
          m_selector,
          m_algaeIntake,
          m_elevator,
          m_dumper,
          m_coDriverController,
          m_vision);
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
      new IntakeAlgaeCmd(
          m_algaeIntake, m_leds, m_elevator, desiredHeight.ALGAELOW, m_driverController);
  private final IntakeAlgaeCmd intakeAlgaeL2 =
      new IntakeAlgaeCmd(m_algaeIntake, m_leds, m_elevator, desiredHeight.ALGAEL2);
  private final IntakeAlgaeCmd intakeAlgaeL3 =
      new IntakeAlgaeCmd(m_algaeIntake, m_leds, m_elevator, desiredHeight.ALGAEL3);
  private final ShootAlgaeCmd shootAlgae = new ShootAlgaeCmd(m_algaeIntake, m_elevator, m_leds);
  private final NetAlgaeShootCmd shootAlgaeNet =
      new NetAlgaeShootCmd(m_algaeIntake, m_leds, m_elevator, m_swerve);

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
  private final ReadyClimbCmd readyclimb = new ReadyClimbCmd(m_climber, m_leds, m_algaeIntake);

  private final AutoDump dumpAuto = new AutoDump(m_dumper);
  private final AutoFeeder feed = new AutoFeeder(m_elevator, m_shooter, m_leds);
  private final AutoFeast cycleToFeeder =
      new AutoFeast(m_swerve, m_elevator, m_shooter, m_leds, m_pathfinding);
  private final AutoCancel cancelAuto =
      new AutoCancel(m_elevator, m_shooter, m_leds, m_algaeIntake);
  private final DeepClimbCmd deepclimb = new DeepClimbCmd(m_climber, m_leds);

  private boolean m_debug = false;

  public RobotContainer() {
    // warms up the pathfinding so that the first path calculation is faster
    PathfindingCommand.warmupCommand();

    setup.setUpDashboardComp();
    if (m_debug) {
      setup.setUpDashboardDebug();
    }

    NamedCommands.registerCommand("dumper", dumpAuto);
    NamedCommands.registerCommand("feed", feed);
    // Pathfinding.configurePathfinder(m_shooter, m_swerve, m_elevator, m_algaeIntake, m_dumper);
    // Auto.initAutoWidget();

    m_swerve.resetModulesToAbsolute();

    m_climber.setDefaultCommand(
        m_climber.deepClimb(
            () ->
                Joysticks.conditionJoystick(
                    () -> m_coDriverController.getLeftY(),
                    climberLimiter,
                    Constants.stickDeadband,
                    true)));

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
            () -> false,
            () -> m_elevator.getEncoderPos());

    m_swerve.setDefaultCommand(teleopCmd);
  }

  public void configureBindingsTest() {
    if (m_debug) {
      setup.setUpDashboardSubsystemTest();
    }
  }

  public void configureBindingsTeleop() {

    m_coDriverController.start().and(m_coDriverController.back()).onTrue(readyclimb);
    // m_coDriverController.leftBumper().onTrue(deepclimb); TODO mettre un vrai boutton

    m_driverController.x().onTrue(intakeAlgaeFloor);

    m_driverController.a().onTrue(intakeCoral);
    m_driverController.b().onTrue(shootAlgae);

    m_coDriverController.a().onTrue(shootCoral);
    // m_driverController.y().onTrue(shootAlgaeNet).onFalse(cancelAuto);
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
                .unless(m_climber::isClimberActivated)
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

    m_coDriverController.rightBumper().onTrue(intakeCoral);

    m_driverController.rightTrigger(0.5).whileTrue(cycleToFeeder).whileFalse(cancelAuto);
  }

  public void teleopInit() {
    // Running this in case our Auto sequence got cancelled early.
    m_swerve.regularConstraints();
  }

  public Command getAutonomousCommand() {
    return m_pathfinding.auto();
  }
}
