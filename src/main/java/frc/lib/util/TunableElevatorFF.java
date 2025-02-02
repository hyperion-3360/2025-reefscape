// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.controller.ElevatorFeedforward;

/** Add your docs here. */
public class TunableElevatorFF /*implements Sendable*/ {
  private double k_a = 0.0;
  private double k_g = 0.0;
  private double k_v = 0.0;
  private final double k_s = 0.1;
  private ElevatorFeedforward m_feedForward;

  // private static int instances = 0;

  public TunableElevatorFF(double g, double v, double a) {
    k_a = a;
    k_g = g;
    k_v = v;
    renewFF();
    // SendableRegistry.add(this, "TunableElevatorFF", instances);
  }

  public double getA() {
    return m_feedForward.getKa();
  }

  public void setA(double a) {
    k_a = a;
    renewFF();
  }

  public double getG() {
    return m_feedForward.getKg();
  }

  public void setG(double g) {
    k_g = g;
    renewFF();
  }

  public double getV() {
    return m_feedForward.getKv();
  }

  public void setV(double v) {
    k_v = v;
    renewFF();
  }

  public void reset() {
    renewFF();
  }

  private void renewFF() {
    m_feedForward = new ElevatorFeedforward(k_s, k_g, k_v, k_a);
  }

  public double calculate(double currentVelocity, double nextVelocity) {
    return m_feedForward.calculateWithVelocities(currentVelocity, nextVelocity);
  }

  // @Override
  // public void initSendable(SendableBuilder builder) {
  //   System.out.println("initSendable, TunableElevatorFF");
  //   builder.setSmartDashboardType("RobotPreferences");
  //   builder.addDoubleProperty("Kv", this::getV, this::setV);
  //   builder.addDoubleProperty("Kg", this::getG, this::setG);
  //   builder.addDoubleProperty("Ka", this::getA, this::setA);
  // }
}
