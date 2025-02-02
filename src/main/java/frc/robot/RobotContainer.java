// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.Auto;
import frc.robot.Auto.Pathfinding;
import frc.robot.commands.DumperCMD;
import frc.robot.commands.IntakeCoralCmd;
import frc.robot.commands.ShootCoralCmd;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Dumper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shootSpeed;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.Patterns;
import frc.robot.subsystems.swerve.CTREConfigs;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.Vision;

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

  // Joystick axis declarations
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int leftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
  private final int rightTriggerAxis = XboxController.Axis.kRightTrigger.value;

  // Slew Rate Limiters to limit acceleration of joystick inputs
  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

  private final SlewRateLimiter elevatorLimiter = new SlewRateLimiter(3);

  // private final SlewRateLimiter clawAngleLimiter = new SlewRateLimiter(3);
  // private final SlewRateLimiter clawPincerLimiter = new SlewRateLimiter(3);

  private final SlewRateLimiter climberSpeedLimiter = new SlewRateLimiter(3);

  private final SlewRateLimiter shooterLimiter = new SlewRateLimiter(3);

  private final double kJoystickDeadband = 0.1;

  public final IntakeCoralCmd intakeCoral = new IntakeCoralCmd(m_shooter, m_leds);
  public final ShootCoralCmd shootCoralL1 = new ShootCoralCmd(m_shooter, m_leds, shootSpeed.L1);
  public final ShootCoralCmd shootCoralL2 = new ShootCoralCmd(m_shooter, m_leds, shootSpeed.L2);
  public final ShootCoralCmd shootCoralL3 = new ShootCoralCmd(m_shooter, m_leds, shootSpeed.L3);
  public final ShootCoralCmd shootCoralL4 = new ShootCoralCmd(m_shooter, m_leds, shootSpeed.L4);

  /***
   * conditionJoystick
   * Condition a joystick axis value given a slewrate limiter and deadband
   * @param axis axis to condition
   * @param limiter slewrate limiter (to smooth the rate of changed
   * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html)
   * @param deadband deadband to suppress noise around the 0 of a joystick axis
   * @see https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/MathUtil.html#applyDeadband(double,double)
   * @return the conditioned value
   */
  private double conditionJoystick(int axis, SlewRateLimiter limiter, double deadband) {
    return Math.pow(
        -limiter.calculate(MathUtil.applyDeadband(m_driverController.getRawAxis(axis), deadband)),
        3);
  }

  public RobotContainer() {

    Auto.initAutoWidget();
    Pathfinding.configurePathfinder(m_shooter, m_swerve, m_elevator, m_algaeIntake, m_dumper);

    m_swerve.resetModulesToAbsolute();
  }

  public void configureBindingsTest() {

    // Swerve control Right bumper + left and right joystick
    m_driverController
        .rightBumper()
        .whileTrue(
            new TeleopSwerve(
                m_swerve,
                () -> conditionJoystick(translationAxis, translationLimiter, kJoystickDeadband),
                () -> conditionJoystick(strafeAxis, strafeLimiter, kJoystickDeadband),
                () -> conditionJoystick(rotationAxis, rotationLimiter, kJoystickDeadband),
                () -> true));

    // Elevator control Pov UP + left joystick
    m_driverController
        .povUp()
        .whileTrue(
            m_elevator.manualTest(() -> -conditionJoystick(translationAxis, elevatorLimiter, 0.0)));
    m_driverController.povUp().onTrue(new DumperCMD(m_dumper));

    // Elevator position BACK and A B X Y for respectively L1 L2 L3 L4
    m_driverController
        .back()
        .and(m_driverController.a())
        .onTrue(m_elevator.Elevate(desiredHeight.LOW));
    m_driverController
        .back()
        .and(m_driverController.b())
        .onTrue(m_elevator.Elevate(desiredHeight.FEEDER));
    m_driverController
        .back()
        .and(m_driverController.x())
        .onTrue(m_elevator.Elevate(desiredHeight.L3));
    m_driverController
        .back()
        .and(m_driverController.y())
        .onTrue(m_elevator.Elevate(desiredHeight.L4));

    // Path finding  START and POV center
    m_driverController
        .start()
        .and(m_driverController.povCenter())
        .onTrue(Pathfinding.doPathfinding());

    // Coral shooter manual test POV Right and left joystick
    m_driverController
        .povRight()
        .whileTrue(
            m_shooter.manualTest(
                () -> conditionJoystick(translationAxis, shooterLimiter, kJoystickDeadband)))
        .onFalse(m_shooter.manualTest(() -> 0.0));

    // Algea elevator manual control  POV left + left joystick
    m_driverController
        .povLeft()
        .whileTrue(
            m_algaeIntake.setAngle(
                () -> conditionJoystick(strafeAxis, strafeLimiter, kJoystickDeadband)))
        .onFalse(m_algaeIntake.setAngle(() -> 0.0));

    // Algea intake manual control POV down + left joystick
    m_driverController
        .povDown()
        .whileTrue(
            m_algaeIntake.setSpeed(
                () -> conditionJoystick(translationAxis, translationLimiter, kJoystickDeadband)))
        .onFalse(m_algaeIntake.setSpeed(() -> 0.0));

    // // Climber manual control left bumpber + left joystick
    m_driverController
        .leftBumper()
        .whileTrue(
            m_climber.climberTestMode(
                () -> conditionJoystick(translationAxis, climberSpeedLimiter, kJoystickDeadband)));

    m_driverController.povUp().onTrue(intakeCoral);
    m_driverController.povDown().onTrue(shootCoralL1);
  }

  public void configureBindingsTeleop() {
    /**
     * conditionJoysitck this is an example of how to assign button :
     * m_driverController.a().onTrue(ALGAE_INTAKE_AUTO); (so clean i know)
     */
    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> conditionJoystick(translationAxis, translationLimiter, kJoystickDeadband),
            () -> conditionJoystick(strafeAxis, strafeLimiter, kJoystickDeadband),
            () -> conditionJoystick(rotationAxis, rotationLimiter, kJoystickDeadband),
            () -> true));
  }

  public Command getAutonomousCommand() {
    return Pathfinding.fullControl();
  }
}
