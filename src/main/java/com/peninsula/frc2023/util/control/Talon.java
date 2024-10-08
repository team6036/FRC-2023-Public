package com.peninsula.frc2023.util.control;

import static java.util.Map.entry;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.esotericsoftware.minlog.Log;
import com.peninsula.frc2023.robot.HardwareWriter;
import java.util.Map;

public class Talon extends TalonSRX implements Controller {

  static class BaseTalonController<T extends BaseTalon & Controller>
      extends ProfiledControllerBase<T> {

    protected double mPositionConversion, mVelocityConversion;

    protected BaseTalonController(T talon) {
      super(talon);
    }

    @Override
    protected void updateGains(
        boolean isFirstInitialization, int slot, Gains newGains, Gains lastGains) {
      super.updateGains(isFirstInitialization, slot, newGains, lastGains);
      if (isFirstInitialization) {
        mController.configMotionSCurveStrength(4, HardwareWriter.kTimeoutMs);
      }
    }

    @Override
    void setProfiledAcceleration(int slot, double acceleration) {
      mController.configMotionAcceleration(round(acceleration), HardwareWriter.kTimeoutMs);
    }

    @Override
    void setProfiledCruiseVelocity(int slot, double cruiseVelocity) {
      mController.configMotionCruiseVelocity(round(cruiseVelocity), HardwareWriter.kTimeoutMs);
    }

    @Override
    protected void setProfiledAllowableError(int slot, double allowableError) {
      mController.configAllowableClosedloopError(
          slot, round(allowableError), HardwareWriter.kTimeoutMs);
    }

    @Override
    protected void setProfiledMinimumVelocityOutput(int slot, double minimumOutputVelocity) {
      // Not supported by Talons
    }

    @Override
    protected boolean setReference(
        ControllerOutput.Mode mode, int slot, double reference, double arbitraryPercentOutput) {
      ControlMode controllerMode = kModeToController.get(mode);
      double convertedReference;
      switch (mode) {
        case VELOCITY:
        case PROFILED_VELOCITY:
          convertedReference = reference / mVelocityConversion;
          break;
        case POSITION:
        case PROFILED_POSITION:
          convertedReference = reference / mPositionConversion;
          break;
        default:
          convertedReference = reference;
          break;
      }
      mController.selectProfileSlot(slot, HardwareWriter.kPidIndex);
      mController.set(
          controllerMode,
          convertedReference,
          DemandType.ArbitraryFeedForward,
          arbitraryPercentOutput);
      return true;
    }

    @Override
    protected void setP(int slot, double p) {
      mController.config_kP(slot, p, HardwareWriter.kTimeoutMs);
    }

    @Override
    protected void setI(int slot, double i) {
      mController.config_kI(slot, i, HardwareWriter.kTimeoutMs);
    }

    @Override
    protected void setD(int slot, double d) {
      mController.config_kD(slot, d, HardwareWriter.kTimeoutMs);
    }

    @Override
    protected void setF(int slot, double f) {
      mController.config_kF(slot, f, HardwareWriter.kTimeoutMs);
    }

    @Override
    protected void setIZone(int slot, double iZone) {
      mController.config_IntegralZone(slot, round(iZone), HardwareWriter.kTimeoutMs);
    }

    @Override
    protected void setIMax(int slot, double iMax) {
      mController.configMaxIntegralAccumulator(slot, iMax, HardwareWriter.kTimeoutMs);
    }

    @Override
    protected void setFrameTimings() {
      /* Update period of commands sent to controller */
      mController.setControlFramePeriod(ControlFrame.Control_3_General, mControlFrameMs);
      /* Update period of feedback received from controller */
      // Applied motor output, fault information, limit switch information
      mController.setStatusFramePeriod(
          StatusFrame.Status_1_General, mStatusFrameMs, HardwareWriter.kTimeoutMs);
      // Selected sensor position and velocity, supply current measurement, sticky fault information
      mController.setStatusFramePeriod(
          StatusFrame.Status_2_Feedback0, mStatusFrameMs, HardwareWriter.kTimeoutMs);
    }
  }

  protected static final Map<ControllerOutput.Mode, ControlMode> kModeToController =
      Map.ofEntries(
          entry(ControllerOutput.Mode.PERCENT_OUTPUT, ControlMode.PercentOutput),
          entry(ControllerOutput.Mode.POSITION, ControlMode.Position),
          entry(ControllerOutput.Mode.VELOCITY, ControlMode.Velocity),
          entry(ControllerOutput.Mode.PROFILED_POSITION, ControlMode.MotionMagic),
          entry(ControllerOutput.Mode.PROFILED_VELOCITY, ControlMode.MotionProfile));
  private final BaseTalonController<Talon> mController = new BaseTalonController<>(this);
  private final String mName;

  public Talon(int deviceId, String name) {
    super(deviceId);
    mName = name;
    clearStickyFaults(HardwareWriter.kTimeoutMs);
  }

  public boolean setOutput(ControllerOutput output) {
    return mController.setOutput(output);
  }

  /**
   * When controllers reset over CAN, frame periods are cleared. This handles resetting them to
   * their configured values before.
   */
  public void handleReset() {
    if (hasResetOccurred()) {
      Log.error("reset", String.format("%s reset", mController.getName()));
      mController.updateFrameTimings();
    }
  }

  public void configFrameTimings(int controlFrameMs, int statusFrameMs) {
    mController.configFrameTimings(controlFrameMs, statusFrameMs);
  }

  public String getName() {
    return String.format("(Talon #%d), %s", getDeviceID(), mName);
  }

  public static int round(double d) {
    return (int) Math.round(d);
  }
}
