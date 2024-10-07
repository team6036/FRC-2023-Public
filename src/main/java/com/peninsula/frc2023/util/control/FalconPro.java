package com.peninsula.frc2023.util.control;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.controls.*;
import com.ctre.phoenixpro.hardware.TalonFX;

public class FalconPro extends TalonFX {

  private String mName;
  private double lastRef = -381904;
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOCSetter =
      new VelocityTorqueCurrentFOC(0);
  private final PositionVoltage positionVoltageSetter = new PositionVoltage(0);
  private final MotionMagicDutyCycle motionMagicTorqueCurrentFOCSetter =
      new MotionMagicDutyCycle(0);
  private final NeutralOut neutralOutSetter = new NeutralOut();
  private final DutyCycleOut dutyCycleOutSetter = new DutyCycleOut(0, true, true);
  private final VoltageOut voltageOutSetter = new VoltageOut(0, true, true);
  private final StatusSignalValue<Double> rotorPositionStatus = getRotorPosition();
  private final StatusSignalValue<Double> rotorVelocityStatus = getRotorVelocity();

  public FalconPro(int deviceNumber, String name, String canbus) {
    super(deviceNumber, canbus);
    this.mName = name;
  }

  public FalconPro(int deviceNumber, String name) {
    super(deviceNumber);
    this.mName = name;
  }

  public void setOutput(ControllerOutput outputs, boolean resetGains) {

    ControllerOutput.Mode mode = outputs.getControlMode();
    double demand = outputs.getArbitraryDemand();
    Gains gains = outputs.getGains();
    double reference = outputs.getReference();

    if (gains != null && resetGains) {
      setGain(gains);
    }

    switch (mode) {
      case VELOCITY:
      case PROFILED_VELOCITY:
        setControl(velocityTorqueCurrentFOCSetter.withVelocity(reference)); // RPS
        break;
      case PERCENT_OUTPUT:
        set(reference);
        break;
      case POSITION:
        setControl(positionVoltageSetter.withPosition(reference)); // Rotation
        break;
      case PROFILED_POSITION:
        setControl(
            motionMagicTorqueCurrentFOCSetter.withPosition(reference).withFeedForward(demand));
        break;
      case BRAKE:
        setControl(neutralOutSetter);
        break;
      case VOLTAGE:
        setControl(dutyCycleOutSetter.withOutput(reference / 12.0));
        break;
    }
    lastRef = reference;
  }

  public String getName() {
    return mName;
  }

  /**
   * Sets the target velocity for motion magic
   *
   * @param sensorUnitsPer100ms Velocity in rotation of target velocity
   */
  public void setMotionCruiseVelocity(double sensorUnitsPer100ms) {
    MotionMagicConfigs m = new MotionMagicConfigs();
    m.MotionMagicCruiseVelocity = sensorUnitsPer100ms;
    getConfigurator().apply(m);
  }

  /**
   * Sets the target acceleration for motion magic
   *
   * @param sensorUnitsPer100ms Acceleration in rotation of target velocity
   */
  public void setMotionAcceleration(double sensorUnitsPer100ms) {
    MotionMagicConfigs m = new MotionMagicConfigs();
    m.MotionMagicAcceleration = sensorUnitsPer100ms;
    getConfigurator().apply(m);
  }

  /**
   * Sets the target velocity and acceleration for motion magic. <br>
   * Same as calling setMotionCruiseVelocity and setMotionAcceleration in effect, but creates 1 less
   * MotionMagicConfigs object
   *
   * @param acceleration Acceleration in rotation of target velocity
   * @param velocity Velocity in rotation of target velocity
   */
  public void setMotionAccelerationAndVelocity(double acceleration, double velocity) {
    MotionMagicConfigs m = new MotionMagicConfigs();
    m.MotionMagicAcceleration = acceleration;
    m.MotionMagicCruiseVelocity = velocity;
    getConfigurator().apply(m);
  }

  /**
   * Sets the PIDF of the motor, applies to slot0 of configurator
   *
   * @param gains
   */
  public void setGain(Gains gains) {
    Slot0Configs configs = new Slot0Configs();
    configs.kP = gains.p;
    configs.kI = gains.i;
    configs.kD = gains.d;
    configs.kV = gains.f;

    getConfigurator().apply(configs, 50);
  }

  /**
   * Set a master motor. Slave motor follows the motions of master motor. <br>
   * Same thing as setControl(new Follower(master.getDeviceID(), setInverse));
   *
   * @param master Motor to follow
   * @param setInverse If slave motor inverse of master.
   */
  public void follow(FalconPro master, boolean setInverse) {
    setControl(new Follower(master.getDeviceID(), setInverse));
  }

  /**
   * Refreshes the rotor position status signal object and returns the value. DOES NOT construct a
   * new status signal object. <br>
   * </br> Same as: rotorPositionStatus.refresh().getValue();
   *
   * @return Rotor Position in rotations
   */
  public double getRotorPositionValue() {
    return rotorPositionStatus.refresh().getValue();
  }

  /**
   * Refreshes the rotor velocity status signal object and returns the value. DOES NOT construct a
   * new status signal object. <br>
   * </br> Same as: rotorVelocityStatus.refresh().getValue();
   *
   * @return Rotor Velocity in rotations per second
   */
  public double getRotorVelocityValue() {
    return rotorVelocityStatus.refresh().getValue();
  }
}
