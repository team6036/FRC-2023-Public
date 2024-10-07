package com.peninsula.frc2023.config;

import com.peninsula.frc2023.util.control.Gains;

public class IntakeConstants {
  public static double intakeRunVolts = 6.0;

  /** Actuation * */
  public static double gearRatio = 30 * 36 / 32.0;

  public static double downSetpointRot = (2.0 / 360.0) * gearRatio;
  public static double stowedSetpoint = (90 / 360.0) * gearRatio;

  /* Motion magic */
  public static Gains intakeGains = new Gains(0.1, 0, 0, 0, 0, 0);

  public static double maxSpeedRotPS = 2.0 * gearRatio;
  public static double maxAccelRotPS2 = 10.0 * gearRatio;
}
