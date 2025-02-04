// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.Joysticks;
import frc.lib.util.TestBindings;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SubsystemInfo;
import frc.robot.Constants.climberAction;
import java.util.function.DoubleSupplier;

/** 1 falcon winch and unwinch* */
// La classe devrait avoir un option pour seulement s'activer a 30sec de la fin du jeu,
// pour eviter un mouvement par accident du pilot
public class Climber extends SubsystemBase implements TestBindings {

  private double kDt = ClimberConstants.kDt;
  private double kMaxVelocity = ClimberConstants.kMaxAcceleration;
  private double kMaxAcceleration = ClimberConstants.kMaxAcceleration;
  private double kP = ClimberConstants.kP;
  private double kI = ClimberConstants.kI;
  private double kD = ClimberConstants.kD;
  private double kS = ClimberConstants.kS;
  private double kG = ClimberConstants.kG;
  private double kV = ClimberConstants.kG;

  private TalonFXConfiguration m_climberMotorConfig = new TalonFXConfiguration();
  private TalonFX m_climberMotor = new TalonFX(SubsystemInfo.kClimberMotorID, "CANivore_3360");

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
  private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV);
  private Double m_direction = 0.0;

  private static double GrabPosition = 30;
  private static double LiftPosition = 0;
  private double m_climberTarget = LiftPosition;
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final SlewRateLimiter climberSpeedLimiter = new SlewRateLimiter(3);

  private TalonFXConfiguration m_climberConfig = new TalonFXConfiguration();

  public Climber() {
    // motor configs
    m_climberMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_climberMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_climberMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    m_climberMotor.getConfigurator().apply(m_climberMotorConfig);

    m_climberMotor.set(0);

    SendableRegistry.add(this, "Climber", 0);
    SmartDashboard.putData("Climber", this);
  }

  // create falcons motor control
  // need function to move with grab/lift
  // pour les falcons utilise talon ex: private WPI_TalonSRX m_exemple = new
  // WPI_TalonSRX(kidexemple);
  public void move(climberAction action) {
    // Take a direction from climberAction enum
    // if needed to block operation before end of game since climber only works once
    // if (DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30)
    switch (action) {
      case GRAB:
        // Set Falcon to go down
        m_climberTarget = GrabPosition;
        break;

      case LIFT:
        // set Falcon to go up
        m_climberTarget = LiftPosition;
        break;
    }
  }

  @Override
  public void periodic() {
    // if (DriverStation.isEnabled()) {
    // Make the motor get to the target
    // m_climberMotor.set(
    // m_controller.calculate(m_climberMotor.getPosition().getValueAsDouble(),
    // m_climberTarget)
    // + m_feedforward.calculate(
    // m_climberMotor.getPosition().getValueAsDouble(),
    // m_climberMotor.getVelocity().getValueAsDouble()));
    // }
    // SmartDashboard.putNumber("ClimbeGrab", GrabPosition);
  }

  public Double getGrabTarget() {
    return GrabPosition;
  }

  public void setGrabTarget(Double value) {
    GrabPosition = value;
  }

  public Double getLiftTarget() {
    return LiftPosition;
  }

  public void setLiftTarget(Double value) {
    LiftPosition = value;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("RobotPreferences");
    builder.addDoubleProperty("GrabTarget", this::getGrabTarget, this::setGrabTarget);
    builder.addDoubleProperty("LiftTarget", this::getLiftTarget, this::setLiftTarget);
  }

  // This is for test mode
  public Command climberTestMode(DoubleSupplier speed) {
    return this.run(
        () -> {
          m_direction = Math.signum(speed.getAsDouble());
          m_climberMotor.set(Math.pow(speed.getAsDouble(), 2) * m_direction);
        });
  }

  @Override
  public void setupTestBindings(Trigger moduleTrigger, CommandXboxController controller) {

    moduleTrigger
        .and(controller.leftBumper())
        .whileTrue(
            this.climberTestMode(
                () ->
                    Joysticks.conditionJoystick(
                        () -> controller.getRawAxis(translationAxis),
                        climberSpeedLimiter,
                        Constants.stickDeadband,
                        true)));
  }
}
