// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.Auto;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.ClimberCmd.ClimberType;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.IntakeCmd.IntakeType;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.ShootCmd.ShootType;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralClaw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.Patterns;
import frc.robot.subsystems.swerve.CTREConfigs;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {

  // controller declarations
  public static final CommandXboxController m_driverController = new CommandXboxController(0);
  public static final CommandXboxController m_coDriverController = new CommandXboxController(1);

  // subsystem declarations
  public static final Vision m_vision = new Vision();
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  public static final Swerve m_swerve = new Swerve(m_vision);
  public static final CoralClaw m_coralClaw = new CoralClaw();
  public static final AlgaeIntake m_algaeIntake = new AlgaeIntake();
  public static final Climber m_climber = new Climber();
  public static final Elevator m_elevator = new Elevator();
  public static final LEDs m_leds = new LEDs();
  public static final Patterns m_patterns = new Patterns();

  // command declarations
  public static final IntakeCmd CORAL_INTAKE_AUTO = new IntakeCmd(IntakeType.CoralAuto);
  public static final IntakeCmd ALGAE_INTAKE_AUTO = new IntakeCmd(IntakeType.AlgaeAuto);
  public static final IntakeCmd ALGAE_INTAKE_GROUND = new IntakeCmd(IntakeType.AlgaeGround);
  // TODO implement coral feeder in IntakeCmd because this crashes sim and is not supported in
  // switch case
  // public static final IntakeCmd CORAL_INTAKE_FEEDER = new IntakeCmd(IntakeType.CoralFeeder);
  public static final ShootCmd CORAL_SHOOT_L1 = new ShootCmd(ShootType.CoralL1);
  public static final ShootCmd CORAL_SHOOT_L2 = new ShootCmd(ShootType.CoralL2);
  public static final ShootCmd CORAL_SHOOT_L3 = new ShootCmd(ShootType.CoralL3);
  public static final ShootCmd CORAL_SHOOT_L4 = new ShootCmd(ShootType.CoralL4);
  public static final ShootCmd CORAL_SHOOT_AUTO = new ShootCmd(ShootType.CoralAuto);
  public static final ShootCmd ALGAE_SHOOT_DITCH = new ShootCmd(ShootType.AlgaeDitch);
  public static final ShootCmd ALGAE_SHOOT_NET = new ShootCmd(ShootType.AlgaeNet);
  public static final ShootCmd ALGAE_SHOOT_PROCESSOR = new ShootCmd(ShootType.AlgaeProcessor);
  public static final ClimberCmd CLIMBER_GRAB = new ClimberCmd(ClimberType.CLIMBERGRAB, m_climber);
  public static final ClimberCmd CLIMBER_LIFT = new ClimberCmd(ClimberType.CLIMBERLIFT, m_climber);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Slew Rate Limiters to limit acceleration of joystick inputs
  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

  private final double kJoystickDeadband = 0.1;

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
    return -limiter.calculate(
        MathUtil.applyDeadband(m_driverController.getRawAxis(axis), deadband));
  }

  public RobotContainer() {
    m_swerve.resetModulesToAbsolute();

    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> conditionJoystick(translationAxis, translationLimiter, kJoystickDeadband),
            () -> conditionJoystick(strafeAxis, strafeLimiter, kJoystickDeadband),
            () -> conditionJoystick(rotationAxis, rotationLimiter, kJoystickDeadband),
            () -> true));

    configureBindings();
    Auto.initAutoWidget();
  }

  private void configureBindings() {
    /**
     * this is an example of how to assign button :
     * m_driverController.a().onTrue(ALGAE_INTAKE_AUTO); (so clean i know)
     */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
