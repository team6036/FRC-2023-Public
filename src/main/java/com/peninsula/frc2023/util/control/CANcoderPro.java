package com.peninsula.frc2023.util.control;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.hardware.CANcoder;

public class CANcoderPro extends CANcoder {
  private StatusSignalValue<Double> mAbsolutePosition = getAbsolutePosition();
  private StatusSignalValue<Double> mPosition = getPosition();

  public CANcoderPro(int deviceID, String canbus) {
    super(deviceID, canbus);
  }

  public double getAbsolutePositionValue() {
    return mAbsolutePosition.refresh().getValue();
  }

  public double getPositionValue() {
    return mPosition.refresh().getValue();
  }
}
