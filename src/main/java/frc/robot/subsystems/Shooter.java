// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.Joysticks;
import frc.lib.util.TestBindings;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase implements TestBindings {
  private WPI_TalonSRX m_shooter = new WPI_TalonSRX(Constants.SubsystemInfo.kCoralShooterTalonID);
  private Servo m_coralBlocker = new Servo(Constants.SubsystemInfo.kCoralShooterServoID);
  private DigitalInput m_shooterIR =
      new DigitalInput(Constants.SubsystemInfo.kCoralShooterBeambreakID);
  private final SlewRateLimiter shooterLimiter = new SlewRateLimiter(3);
  private final int translationAxis = XboxController.Axis.kLeftY.value;

  private boolean getShooterIR = m_shooterIR.get();
  private double m_shooterSpeed = Constants.CoralShooterVariables.kShootNo;

  public enum shootSpeed {
    L1,
    L2,
    L3,
    L4,
    STOP,
    INTAKE,
    OTHER
  }

  private void setSpeed(shootSpeed speed) {

    switch (speed) {
      case L1:
        m_shooterSpeed = Constants.CoralShooterVariables.kShootL1;
        break;

      case L2:
        m_shooterSpeed = Constants.CoralShooterVariables.kShootL2;
        break;

      case L3:
        m_shooterSpeed = Constants.CoralShooterVariables.kShootL3;
        break;

      case L4:
        m_shooterSpeed = Constants.CoralShooterVariables.kShootL4;
        break;

      case STOP:
        m_shooterSpeed = Constants.CoralShooterVariables.kShootNo;
        break;

      case INTAKE:
        m_shooterSpeed = Constants.CoralShooterVariables.kIntakeSpeed;
        break;

      case OTHER:
        m_shooterSpeed = -0.7;

        break;
    }
  }

  /** 1 TalonFX controlling 2 BAGs, 1 Servo and 1 beambreak */
  public Shooter() {

    // Srx config (directly on the srx because srxconfig object is kinda limited)
    m_shooter.configFactoryDefault();
    m_shooter.setNeutralMode(Constants.CoralShooterVariables.kCoralShooterNeutralMode);
    m_shooter.configContinuousCurrentLimit(
        Constants.CoralShooterVariables.kCoralShooterCurrentLimit);
    m_shooter.enableCurrentLimit(false);
  }

  public void stop() {
    m_shooterSpeed = 0.0;
  }

  public void openBlocker() {
    m_coralBlocker.setAngle(Constants.CoralShooterVariables.kCoralShooterOpen);
  }

  public void closeBlocker() {
    m_coralBlocker.setAngle(Constants.CoralShooterVariables.kCoralShooterClosed);
  }

  @Override
  public void periodic() {
    getShooterIR = m_shooterIR.get();
    if (DriverStation.isDisabled()) {
      m_shooter.set(0.0);
      closeBlocker();
    } else m_shooter.set(m_shooterSpeed);
    if (Climber.climberActivated()) {
      return;
    }
  }

  /**
   * this method is used when manually testing the shooter using joysticks
   *
   * @param joystick the position of the joystick to influence motor speed
   * @return a command that runs the motor at a speed equivalent to the joystick position squared
   */
  public Command manualTest(DoubleSupplier joystick) {
    // the speed input is squard to have a smoother control curve

    return this.run(
        () -> {
          double throttle = joystick.getAsDouble();
          m_shooter.set(-1.0 * throttle * throttle);
        });
  }

  /** We want a ramp rate when intaking so set the speed at -1 in 0.5 seconds */
  public void setIntake() {
    m_shooter.set(Constants.CoralShooterVariables.kIntakeSpeed);
  }

  /** We don't want a ramp rate when shooting so set the speed at -1 in 0.0 seconds */
  public void setShoot(shootSpeed speed) {
    setSpeed(speed);
  }

  public boolean isCoralIn() {
    return !getShooterIR;
  }

  @Override
  public void setupTestBindings(Trigger moduleTrigger, CommandXboxController controller) {
    moduleTrigger
        .and(controller.x())
        .onTrue(Commands.runOnce(() -> this.openBlocker(), this))
        .onFalse(Commands.runOnce(() -> this.closeBlocker(), this));

    moduleTrigger
        .and(controller.a())
        .whileTrue(
            this.manualTest(
                () ->
                    Joysticks.conditionJoystick(
                        () -> controller.getRawAxis(translationAxis),
                        shooterLimiter,
                        Constants.stickDeadband,
                        true)))
        .onFalse(this.manualTest(() -> 0.0));
  }
}
