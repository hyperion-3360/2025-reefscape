// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.Joysticks;
import frc.robot.Auto.PathfindingV2;
import frc.robot.commands.AlignPeg;
import frc.robot.commands.AlignPeg.Direction;
import frc.robot.commands.AutoCmd.AutoCancel;
import frc.robot.commands.AutoCmd.AutoCancelNet;
import frc.robot.commands.AutoCmd.AutoDump;
import frc.robot.commands.AutoCmd.AutoFeast;
import frc.robot.commands.AutoCmd.AutoFeeder;
import frc.robot.commands.CancelAlignPeg;
import frc.robot.commands.DeepClimbCmd;
import frc.robot.commands.DriveAndIntakeCmd;
import frc.robot.commands.ElevateCmd;
import frc.robot.commands.IntakeAlgaeCmd;
import frc.robot.commands.IntakeCoralCmd;
import frc.robot.commands.LowerElevatorCmd;
import frc.robot.commands.MinuteMoveCmd;
import frc.robot.commands.MinuteMoveCmd.OffsetDir;
import frc.robot.commands.NetAlgaeShootCmd;
import frc.robot.commands.RainbowLedsCmd;
import frc.robot.commands.ReReadyClimbCmd;
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
import frc.robot.vision.PegDetect;
import frc.robot.vision.Vision;
import java.util.Set;

public class RobotContainer {

  // controller declarations
  public static final CommandXboxController m_driverController = new CommandXboxController(0);
  public static final CommandXboxController m_coDriverController = new CommandXboxController(1);
  //  public static final CommandXboxController m_testController = new CommandXboxController(2);

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
  public PathfindingV2 m_pathfinding;

  public static ElasticSetup m_ElasticSetup;

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
  private final DriveAndIntakeCmd intakeReef =
      new DriveAndIntakeCmd(
          m_algaeIntake, m_leds, m_elevator, desiredHeight.ALGAEL2, m_swerve, m_vision);

  private final IntakeAlgaeCmd intakeAlgaeLollypop =
      new IntakeAlgaeCmd(m_algaeIntake, m_leds, m_elevator, desiredHeight.LOLLYPOP);

  private final ShootAlgaeCmd shootAlgae = new ShootAlgaeCmd(m_algaeIntake, m_elevator, m_leds);
  private final NetAlgaeShootCmd shootAlgaeNet;

  private final ShootCoralCmd shootCoral = new ShootCoralCmd(m_shooter, m_leds, m_elevator);
  private final IntakeCoralCmd intakeCoral = new IntakeCoralCmd(m_shooter, m_elevator, m_leds);

  private final AutoCancelNet netCancel;

  private final ElevateCmd elevateL2 =
      new ElevateCmd(m_elevator, m_shooter, m_algaeIntake, m_leds, desiredHeight.L2);
  private final ElevateCmd elevateL3 =
      new ElevateCmd(m_elevator, m_shooter, m_algaeIntake, m_leds, desiredHeight.L3);
  private final ElevateCmd elevateL4 =
      new ElevateCmd(m_elevator, m_shooter, m_algaeIntake, m_leds, desiredHeight.L4);
  private final LowerElevatorCmd elevateLOW =
      new LowerElevatorCmd(m_elevator, m_leds, m_shooter, m_algaeIntake, m_swerve);
  private final LowerElevatorCmd stopIntakeAlgaeReef =
      new LowerElevatorCmd(m_elevator, m_leds, m_shooter, m_algaeIntake, m_swerve);
  private final ReadyClimbCmd readyclimb = new ReadyClimbCmd(m_climber, m_leds, m_algaeIntake);
  private final RainbowLedsCmd rainbow = new RainbowLedsCmd(m_leds);
  private final AutoDump dumpAuto = new AutoDump(m_dumper);
  private final AutoFeeder feed = new AutoFeeder(m_elevator, m_shooter, m_leds);
  private final AutoFeast cycleToFeeder;
  private final AutoCancel cancelAuto =
      new AutoCancel(m_elevator, m_shooter, m_leds, m_algaeIntake);
  private final DeepClimbCmd deepclimb = new DeepClimbCmd(m_climber, m_leds);
  private final ReReadyClimbCmd unguckClimb = new ReReadyClimbCmd(m_climber);
  private final MinuteMoveCmd MinutieMoveLeft =
      new MinuteMoveCmd(m_swerve, 0.5, 0.05, OffsetDir.LEFT);
  private final MinuteMoveCmd MinutieMoveRight =
      new MinuteMoveCmd(m_swerve, 0.5, 0.05, OffsetDir.RIGHT);
  private final MinuteMoveCmd MinutieMoveFront =
      new MinuteMoveCmd(m_swerve, 0.5, 0.05, OffsetDir.FRONT);
  private final MinuteMoveCmd MinutieMoveBack =
      new MinuteMoveCmd(m_swerve, 0.5, 0.05, OffsetDir.BACK);

  private final CancelAlignPeg cancelAlignPeg = new CancelAlignPeg(m_swerve, m_shooter, m_elevator);
  //   private final DriveToSomeTargetCmd backPose =
  //       new DriveToSomeTargetCmd(() -> new Pose2d(7.33, 1.78, Rotation2d.fromDegrees(270)),
  // m_swerve);
  //   private final DriveToSomeTargetCmd leftPose =
  //       new DriveToSomeTargetCmd(() -> new Pose2d(6.55, 3.9, Rotation2d.fromDegrees(270)),
  // m_swerve);
  //   private final DriveToSomeTargetCmd frontPose =
  //       new DriveToSomeTargetCmd(() -> new Pose2d(5.37, 1.73, Rotation2d.fromDegrees(270)),
  // m_swerve);

  private final MinuteMoveCmd MinutieMoveLeftPeg =
      new MinuteMoveCmd(m_swerve, OffsetDir.LEFT, m_algaeIntake);
  private final MinuteMoveCmd MinutieMoveRightPeg =
      new MinuteMoveCmd(m_swerve, OffsetDir.RIGHT, m_algaeIntake);

  private Command m_autoThreeCoralLeftAuto;
  private Command m_autoThreeCoralRightAuto;
  private Command m_autoOneCoralThenAlgae;
  private Command m_autoLine;
  private PegDetect m_pegDetect;

  private final AlignPeg alignPegLeft;

  private final AlignPeg alignPegRight;

  private boolean m_debug = false;
  private UsbCamera m_camera;

  public RobotContainer() {

    SmartDashboard.putData("Main scheduler", CommandScheduler.getInstance());

    m_camera = CameraServer.startAutomaticCapture();
    m_camera.setFPS(45);
    m_camera.setResolution(160, 100);

    m_pegDetect = new PegDetect(CameraServer.getVideo(m_camera));
    m_pathfinding =
        new PathfindingV2(
            m_shooter, m_elevator, m_leds, m_swerve, m_algaeIntake, m_pegDetect, m_vision);

    m_pathfinding =
        new PathfindingV2(
            m_shooter, m_elevator, m_leds, m_swerve, m_algaeIntake, m_pegDetect, m_vision);
    alignPegLeft =
        new AlignPeg(
            m_swerve, m_elevator, m_shooter, m_algaeIntake, m_pegDetect, m_vision, Direction.left);
    alignPegRight =
        new AlignPeg(
            m_swerve, m_elevator, m_shooter, m_algaeIntake, m_pegDetect, m_vision, Direction.right);
    shootAlgaeNet =
        new NetAlgaeShootCmd(m_algaeIntake, m_leds, m_elevator, m_swerve, m_pathfinding);
    netCancel = new AutoCancelNet(m_algaeIntake, m_leds, m_elevator, m_swerve, m_pathfinding);
    cycleToFeeder = new AutoFeast(m_swerve, m_elevator, m_shooter, m_leds, m_pathfinding);
    m_ElasticSetup =
        new ElasticSetup(
            m_swerve,
            m_shooter,
            m_climber,
            m_algaeIntake,
            m_elevator,
            m_coDriverController,
            m_vision,
            m_pathfinding);

    m_ElasticSetup.setUpDashboardComp();
    if (m_debug) {
      m_ElasticSetup.setUpDashboardDebug();
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

  public void configureAutos() {

    // warms up the pathfinding so that the first path calculation is faster
    PathfindingCommand.warmupCommand();

    m_autoThreeCoralLeftAuto = m_pathfinding.ThreeCoralLeft();
    m_autoThreeCoralRightAuto = m_pathfinding.ThreeCoralRight();
    m_autoOneCoralThenAlgae = m_pathfinding.coralAndAlgae();
    m_autoLine = m_pathfinding.straightLine();
  }

  public void configureBindingsTest() {
    if (m_debug) {
      m_ElasticSetup.setUpDashboardSubsystemTest();
    }
  }

  public void configureBindingsTeleop() {

    m_coDriverController
        .start()
        .and(m_coDriverController.back())
        .and(() -> !m_climber.isClimberActivated())
        .onTrue(readyclimb);
    m_coDriverController
        .leftTrigger()
        .and(m_coDriverController.rightTrigger())
        .and(() -> m_climber.isClimberActivated())
        .onTrue(deepclimb);
    m_coDriverController
        .start()
        .and(m_coDriverController.back())
        .and(() -> m_climber.isClimberActivated())
        .onTrue(unguckClimb);

    m_coDriverController.a().onTrue(shootCoral);

    m_coDriverController
        .povDown()
        .onTrue(intakeAlgaeLollypop)
        .onFalse(intakeAlgaeLollypop.NoAlgaeCmd(m_elevator, m_algaeIntake, m_leds));

    m_coDriverController.povUp().onTrue(elevateL4);
    m_coDriverController.povLeft().onTrue(elevateL3);
    m_coDriverController.povRight().onTrue(elevateL2);
    m_coDriverController.b().onTrue(elevateLOW);
    // m_coDriverController.rightBumper().onTrue(intakeCoral);
    m_coDriverController
        .leftBumper()
        .onTrue(Commands.runOnce(() -> m_elevator.decreaseHeightState()));
    m_coDriverController
        .rightBumper()
        .onTrue(Commands.runOnce(() -> m_elevator.increaseHeightState()));

    m_coDriverController.y().onTrue(intakeCoral);

    m_driverController.x().onTrue(intakeAlgaeFloor);

    m_driverController.b().onTrue(shootAlgae);

    m_driverController
        .start()
        .and(m_driverController.back())
        .onTrue(
            new DeferredCommand(
                () ->
                    new InstantCommand(
                        () -> {
                          shootAlgaeNet.toggleManualMode();
                          intakeReef.toggleManualMode();
                          cycleToFeeder.toggleManualMode();
                        }),
                Set.of()));
    m_driverController.rightTrigger(0.3).onTrue(shootAlgaeNet).onFalse(netCancel);

    m_driverController.leftTrigger(0.3).onTrue(intakeReef).onFalse(stopIntakeAlgaeReef);

    m_driverController.povLeft().onTrue(MinutieMoveLeft);
    m_driverController.povRight().onTrue(MinutieMoveRight);
    m_driverController.povUp().onTrue(MinutieMoveFront);
    m_driverController.povDown().onTrue(MinutieMoveBack);

    m_coDriverController.x().toggleOnTrue(rainbow);

    m_coDriverController.x().toggleOnFalse(Commands.runOnce(() -> rainbow.end(true)));

    // m_driverController
    //     .leftBumper()
    //     .onTrue(
    //         Commands.runOnce(() -> m_swerve.drivetoTarget(m_vision.getDesiredPoseLeft()))
    //             .unless(m_climber::isClimberActivated)
    //         //          .andThen(new WaitUntilCommand(m_swerve::targetReached).andThen(() ->
    //         // m_leds.SetPattern(LEDs.Pattern.READY)))
    //         //         .raceWith(new WaitUntilCommand(m_swerve::targetDriveDisabled).andThen(()
    // ->
    //         // m_leds.SetPattern(LEDs.Pattern.READY)))
    //         )
    //     .onFalse(Commands.runOnce(() -> m_swerve.disableDriveToTarget()));

    // m_driverController
    //     .rightBumper()
    //     .onTrue(
    //         Commands.runOnce(() -> m_swerve.drivetoTarget(m_vision.getDesiredPoseRight()))

    //         //          .andThen(new WaitUntilCommand(m_swerve::targetReached).andThen(() ->
    //         // m_leds.SetPattern(LEDs.Pattern.READY)))
    //         //         .raceWith(new WaitUntilCommand(m_swerve::targetDriveDisabled).andThen(()
    // ->
    //         // m_leds.SetPattern(LEDs.Pattern.READY)))
    //         )
    //     .onFalse(Commands.runOnce(() -> m_swerve.disableDriveToTarget()));

    m_driverController.leftBumper().onTrue(alignPegLeft).onFalse(cancelAlignPeg);
    m_driverController.rightBumper().onTrue(alignPegRight).onFalse(cancelAlignPeg);

    m_driverController
        .povUp()
        .onTrue(
            new DeferredCommand(
                () ->
                    Commands.runOnce(
                        () ->
                            m_swerve.drivetoTarget(
                                m_vision.getDesiredPoseAlgae(() -> m_swerve.getPose()))),
                Set.of(m_swerve)));
    m_driverController.y().whileTrue(cycleToFeeder).onFalse(cancelAuto);

    //    m_testController.povLeft().onTrue(MinutieMoveLeft);
    //   m_testController.povRight().onTrue(MinutieMoveRight);
    //  m_testController.povUp().onTrue(elevateL4);
    // m_testController
    //     .a()
    //     .onTrue(new AlignPeg(m_swerve, m_elevator, m_pegDetect, m_vision.getDesiredPoseLeft()));
    // m_testController.b().onTrue(shootCoral);
  }

  public void teleopInit() {
    // Running this in case our Auto sequence got cancelled early.
    m_swerve.regularConstraints();
  }

  public Command getAutonomousCommand() {
    var selected = ElasticSetup.SelectedAuto();
    if (selected == Constants.AutoConstants.Sequence.OneCoralThenAlgae) {
      return m_autoOneCoralThenAlgae;
    } else if (selected == Constants.AutoConstants.Sequence.ThreeCoralLeft) {
      return m_autoThreeCoralLeftAuto;
    } else if (selected == Constants.AutoConstants.Sequence.ThreeCoralRight) {
      return m_autoThreeCoralRightAuto;
    } else if (selected == Constants.AutoConstants.Sequence.Line) {
      return m_autoLine;
    } else {
      return Commands.none();
    }
  }
}
