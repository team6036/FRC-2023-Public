package com.peninsula.frc2023.util.config;

import com.ctre.phoenixpro.configs.*;
import com.peninsula.frc2023.util.control.FalconPro;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the
 * application.
 *
 * <p>Author: Team 254 (edit by 6036)
 */
public class FalconFactoryPro {

  private static final int kTimeoutMs = 100;

  // create a CANTalon with the default (out of the box) configuration
  public static FalconPro createFalconPro(int id, TalonFXConfiguration config, String name) {
    FalconPro talon = new FalconPro(id, name, "swerve");
    talon.set(0);

    TalonFXConfigurator d = talon.getConfigurator();

    d.apply(config);

    d.defaultTimeoutSeconds = kTimeoutMs;
    d.clearStickyFaults(kTimeoutMs / 1000.0);
    d.apply(new SoftwareLimitSwitchConfigs());

    return talon;
  }

  public static FalconPro createDefaultFalconPro(int id, String name) {
    return createFalconPro(id, new TalonFXConfiguration(), name);
  }
}
