package com.peninsula.frc2023.util.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Falcon extends TalonFX {

  private String mName;
  private double lastRef = -381904;

  public Falcon(int deviceNumber, String name) {
    super(deviceNumber);
    this.mName = name;
  }

  public void setOutput(ControllerOutput outputs, boolean resetGains) {

    ControllerOutput.Mode mode = outputs.getControlMode();
    double demand = outputs.getArbitraryDemand();
    Gains gains = outputs.getGains();
    double reference = outputs.getReference();

    if (gains != null && resetGains) {
      config_kP(0, gains.p);
      config_kI(0, gains.i);
      config_kD(0, gains.d);
      config_kF(0, gains.f);
    }
    if (reference != lastRef) {
      switch (mode) {
        case VELOCITY:
        case PROFILED_VELOCITY:
          set(ControlMode.Velocity, reference, DemandType.ArbitraryFeedForward, demand);
          break;
        case PERCENT_OUTPUT:
          set(ControlMode.PercentOutput, reference, DemandType.ArbitraryFeedForward, demand);
          break;
        case POSITION:
        case PROFILED_POSITION:
          set(ControlMode.Position, reference, DemandType.ArbitraryFeedForward, demand);
          break;
        case BRAKE:
          break;
      }
      lastRef = reference;
    }
  }

  public String getName() {
    return mName;
  }
}
