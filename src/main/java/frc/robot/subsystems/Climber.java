// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
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
import frc.robot.Constants.SubsystemInfo;
import java.util.function.DoubleSupplier;

/** 1 falcon winch and unwinch* */
// La classe devrait avoir un option pour seulement s'activer a 30sec de la fin du jeu,
// pour eviter un mouvement par accident du pilot
public class Climber extends SubsystemBase implements TestBindings {

  private TalonFXConfiguration m_climberMotorConfig = new TalonFXConfiguration();
  private TalonFX m_deepMotor = new TalonFX(SubsystemInfo.kClimberDeepMotorID);

  private Double m_direction = 0.0;

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final SlewRateLimiter climberSpeedLimiter = new SlewRateLimiter(3);
  private final DigitalInput m_beamBrake =
      new DigitalInput(Constants.SubsystemInfo.kClimberBeamBrakeID);
  private final Servo m_penis =
      new Servo(Constants.SubsystemInfo.kClimberPenisID); // 90 ouvert commence 0
  private final Servo m_finger =
      new Servo(Constants.SubsystemInfo.kClimberFingerID); // 90 est ouvert

  private static boolean isClimberActivated = false;

  public Climber() {
    m_deepMotor.setNeutralMode(NeutralModeValue.Brake);
    // motor configs
    m_climberMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_climberMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_climberMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    m_deepMotor.getConfigurator().apply(m_climberMotorConfig);
    m_deepMotor.set(0);
    m_finger.setAngle(88);
    m_penis.setAngle(0);
    m_deepMotor.setPosition(0.0);
  }

  public void periodic() {}

  public double getEncoderPosition() {
    return m_deepMotor.getPosition().getValueAsDouble();
  }

  public void Penis90() {
    m_penis.setAngle(90);
  }

  public void Penis0() {
    m_penis.setAngle(0);
  }

  public Command deepClimb(DoubleSupplier speed) {
    return this.run(
        () -> {
          double val = speed.getAsDouble();
          double motorspeed = Math.pow(val, 2) * m_direction;
          m_direction = Math.signum(val);
          m_deepMotor.set(motorspeed);
        });
  }

  public void fingerOpen() {
    m_finger.setAngle(88);
  }

  public void fingerClose() {
    m_finger.setAngle(60);
  }

  public void winchDeepClimb() {
    m_deepMotor.set(0.2);
  }

  public void stopDeepClimb() {
    m_deepMotor.set(0);
  }

  public boolean SensorDetected() {
    return m_beamBrake.get();
  }

  @Override
  public void setupTestBindings(Trigger moduleTrigger, CommandXboxController controller) {

    moduleTrigger
        .and(controller.rightBumper())
        .whileTrue(
            this.deepClimb(
                () ->
                    Joysticks.conditionJoystick(
                        () -> controller.getRawAxis(translationAxis),
                        climberSpeedLimiter,
                        Constants.stickDeadband,
                        true)));
  }

  private boolean isAtPose() {
    return m_deepMotor.getPosition().getValueAsDouble() <= -90;
  }

  public Command goForthChild() {
    return Commands.run(() -> m_deepMotor.set(-0.6)) // - dewinch, + winch
        .until(this::isAtPose)
        .andThen(() -> m_deepMotor.set(0));
  }

  public boolean isClimberActivated() {
    return isClimberActivated;
  }

  public void setClimberActivated() {
    isClimberActivated = true;
  }

  public static boolean climberActivated() {
    return isClimberActivated;
  }
}
