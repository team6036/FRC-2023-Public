package com.peninsula.frc2023.util;

// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import com.peninsula.frc2023.config.SwerveConstants;

public final class CTREConfigs {
  public static TalonFXConfiguration swerveDriveFXConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    TorqueCurrentConfigs torqueCurrentConfigs = new TorqueCurrentConfigs();
    torqueCurrentConfigs.PeakForwardTorqueCurrent =
        SwerveConstants.Constants.Swerve.drivePeakCurrentLimit;
    torqueCurrentConfigs.PeakReverseTorqueCurrent =
        -SwerveConstants.Constants.Swerve.drivePeakCurrentLimit;
    config.TorqueCurrent = torqueCurrentConfigs;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = SwerveConstants.Constants.Swerve.driveKP;
    slot0.kI = SwerveConstants.Constants.Swerve.driveKI;
    slot0.kD = SwerveConstants.Constants.Swerve.driveKD;
    slot0.kV = SwerveConstants.Constants.Swerve.driveKF;
    config.Slot0 = slot0;

    OpenLoopRampsConfigs openLoop = new OpenLoopRampsConfigs();
    openLoop.DutyCycleOpenLoopRampPeriod = SwerveConstants.Constants.Swerve.openLoopRamp;
    config.OpenLoopRamps = openLoop;
    ClosedLoopRampsConfigs closeLoop = new ClosedLoopRampsConfigs();
    closeLoop.DutyCycleClosedLoopRampPeriod = SwerveConstants.Constants.Swerve.closedLoopRamp;
    config.ClosedLoopRamps = closeLoop;
    return config;
  }

  public static TalonFXConfiguration swerveAngleFXConfig() {
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = SwerveConstants.Constants.Swerve.angleKP;
    slot0.kI = SwerveConstants.Constants.Swerve.angleKI;
    slot0.kD = SwerveConstants.Constants.Swerve.angleKD;
    slot0.kV = SwerveConstants.Constants.Swerve.angleKF;
    angleConfig.Slot0 = slot0;

    return angleConfig;
  }

  public static CANcoderConfiguration swerveCancoderConfig() {
    CANcoderConfiguration CANconfig = new CANcoderConfiguration();
    MagnetSensorConfigs config = new MagnetSensorConfigs();
    config.SensorDirection =
        SwerveConstants.Constants.Swerve.canCoderInvert
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    CANconfig.MagnetSensor = config;
    return CANconfig;
  }
}
