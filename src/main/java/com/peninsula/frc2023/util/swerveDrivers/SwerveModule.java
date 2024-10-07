package com.peninsula.frc2023.util.swerveDrivers;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.*;
import com.peninsula.frc2023.config.SwerveConstants;
import com.peninsula.frc2023.util.CTREConfigs;
import com.peninsula.frc2023.util.CTREModuleState;
import com.peninsula.frc2023.util.SwerveModuleConstants;
import com.peninsula.frc2023.util.config.FalconFactoryPro;
import com.peninsula.frc2023.util.control.CANcoderPro;
import com.peninsula.frc2023.util.control.FalconPro;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  public int moduleNumber;
  public double angleOffset;
  private FalconPro mAngleMotor;
  private FalconPro mDriveMotor;
  private CANcoderPro angleEncoder;
  private double lastAngle;

  private double anglekP;
  private double anglekI;
  private double anglekD;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          SwerveConstants.Constants.Swerve.driveKS,
          SwerveConstants.Constants.Swerve.driveKV,
          SwerveConstants.Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoderPro(moduleConstants.cancoderID, "swerve");
    configAngleEncoder();

    /* Angle Motor Config */
    mAngleMotor =
        FalconFactoryPro.createDefaultFalconPro(moduleConstants.angleMotorID, "angle motor");
    configAngleMotor();
    TalonFXConfiguration angleConfiguration = CTREConfigs.swerveAngleFXConfig();
    anglekP = angleConfiguration.Slot0.kP;
    anglekI = angleConfiguration.Slot0.kI;
    anglekD = angleConfiguration.Slot0.kD;

    /* Drive Motor Config */
    mDriveMotor =
        FalconFactoryPro.createDefaultFalconPro(moduleConstants.driveMotorID, "drive motor");
    configDriveMotor();

    lastAngle = getState().angle.getDegrees();
  }

  public void setDesiredState(
      SwerveModuleState desiredState, SwerveModuleState currentModuleState, boolean isOpenLoop) {
    //    System.out.println(desiredState.angle.getDegrees());
    desiredState =
        CTREModuleState.optimize(
            desiredState,
            currentModuleState
                .angle); // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which CTRE is not

    if (isOpenLoop) {
      double percentOutput =
          desiredState.speedMetersPerSecond / SwerveConstants.Constants.Swerve.maxSpeed;
      //      mDriveMotor.setControl(new DutyCycleOut(percentOutput));
      mDriveMotor.set(percentOutput);
    } else {
      double velocity =
          Conversions.MPSToFalcon(
              desiredState.speedMetersPerSecond,
              SwerveConstants.Constants.Swerve.wheelCircumference,
              SwerveConstants.Constants.Swerve.driveGearRatio);

      mDriveMotor.setControl(
          new VelocityDutyCycle(
              velocity / 60.0,
              false,
              feedforward.calculate(desiredState.speedMetersPerSecond),
              0,
              false));
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond)
                <= (SwerveConstants.Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents
    // jittering.

    mAngleMotor.setControl(
        new PositionDutyCycle(
            Conversions.degreesToFalcon(
                desiredState.angle.getDegrees(), SwerveConstants.Constants.Swerve.angleGearRatio),
            false,
            0,
            0,
            true));
    lastAngle = angle;
  }

  public boolean check() {
    return driveSet && angleSet && zeroingSet && cancoderSet;
  }

  public void resetToAbsolute() {

    if (zeroingSet) return;

    double absolutePosition =
        Conversions.degreesToFalcon(
            getCanCoder().getDegrees() - angleOffset,
            SwerveConstants.Constants.Swerve.angleGearRatio);
    var setResult = mAngleMotor.setRotorPosition(absolutePosition, 0.02);

    zeroingSet = setResult.isOK();

    System.out.println(
        "---------- ZEROING (" + mAngleMotor.getDeviceID() + ") | ZERO: " + setResult);
    lastAngle = 0;
  }

  public boolean cancoderSet = false;
  public boolean driveSet = false;
  public boolean angleSet = false;
  public boolean zeroingSet = false;

  private void configAngleEncoder() {

    if (cancoderSet) return;

    StatusCode resultDef =
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration()); // Factory Default
    StatusCode resultSwerve =
        angleEncoder.getConfigurator().apply(CTREConfigs.swerveCancoderConfig());

    cancoderSet = resultDef.isOK() && resultSwerve.isOK();

    System.out.println(
        "---------- CANCoder ("
            + angleEncoder.getDeviceID()
            + ") | Default: "
            + resultDef.getName()
            + " | Spec: "
            + resultSwerve.getName());
  }

  private void configAngleMotor() {
    //    mAngleMotor.configFactoryDefault();

    if (angleSet) return;

    StatusCode resultDef = mAngleMotor.getConfigurator().apply(new TalonFXConfiguration());

    MotorOutputConfigs m = new MotorOutputConfigs();
    m.NeutralMode = SwerveConstants.Constants.Swerve.angleNeutralModePro;
    StatusCode resultNeutral = mAngleMotor.getConfigurator().apply(m);

    StatusCode resultSwerve =
        mAngleMotor.getConfigurator().apply(CTREConfigs.swerveAngleFXConfig());
    mAngleMotor.setInverted(SwerveConstants.Constants.Swerve.angleMotorInvert);

    angleSet = resultDef.isOK() && resultNeutral.isOK() && resultSwerve.isOK();

    System.out.println(
        "---------- TalonFX A ("
            + mAngleMotor.getDeviceID()
            + ") | Default: "
            + resultDef.getName()
            + " | Spec: "
            + resultSwerve.getName()
            + " | Net: "
            + resultNeutral.getName());

    SmartDashboard.putBoolean("AngleMod" + moduleNumber, angleSet);

    resetToAbsolute();
  }

  private void configDriveMotor() {
    //    mDriveMotor.configFactoryDefault();

    if (driveSet) return;

    StatusCode resultSwerve =
        mDriveMotor.getConfigurator().apply(CTREConfigs.swerveDriveFXConfig());

    MotorOutputConfigs m = new MotorOutputConfigs();
    m.NeutralMode = SwerveConstants.Constants.Swerve.driveNeutralModePro;
    StatusCode resultNeutral = mDriveMotor.getConfigurator().apply(m);
    mDriveMotor.setInverted(SwerveConstants.Constants.Swerve.driveMotorInvert);

    StatusCode rotorSetDef = mDriveMotor.setRotorPosition(0, 0.05);

    driveSet = resultSwerve.isOK() && resultNeutral.isOK() && rotorSetDef.isOK();

    System.out.println(
        "---------- TalonFX D ("
            + mAngleMotor.getDeviceID()
            + ") | RotorSet: "
            + rotorSetDef.getName()
            + " | Spec: "
            + resultSwerve.getName()
            + " | Net: "
            + resultNeutral.getName());
  }

  public void updateAnglePID(double kP, double kI, double kD) {
    //    if (anglekP != kP) {
    //      anglekP = kP;
    //      mAngleMotor.config_kP(0, anglekP, 100);
    //    }
    //    if (anglekI != kI) {
    //      anglekI = kI;
    //      mAngleMotor.config_kI(0, anglekI, 100);
    //    }
    //    if (anglekD != kP) {
    //      anglekD = kD;
    //      mAngleMotor.config_kD(0, anglekD, 100);
    //    }
    if (anglekP != kP || anglekI != kI || anglekD != kP) {
      anglekP = kP;
      anglekI = kI;
      anglekD = kD;
      Slot0Configs slot0 = new Slot0Configs();
      slot0.kP = kP;
      slot0.kI = kI;
      slot0.kD = kD;
      mAngleMotor.getConfigurator().apply(slot0);
    }
  }

  public double[] getAnglePIDValues() {
    double[] values = {anglekP, anglekI, anglekD};
    return values;
  }

  public Rotation2d getCanCoder() {
    //    var pos = angleEncoder.getAbsolutePosition();
    //    return Rotation2d.fromDegrees(pos.getValue() * 360.0);
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePositionValue() * 360.0);
  }

  public double getTargetAngle() {
    return lastAngle;
  }

  public SwerveModulePosition getPosition() {
    double pos =
        mDriveMotor.getRotorPositionValue()
            * SwerveConstants.Constants.Swerve.wheelCircumference
            / SwerveConstants.Constants.Swerve.driveGearRatio;
    Rotation2d angle =
        Rotation2d.fromDegrees(
            Conversions.falconToDegrees(
                mAngleMotor.getRotorPositionValue(),
                SwerveConstants.Constants.Swerve.angleGearRatio));

    return new SwerveModulePosition(pos, angle);
  }

  public SwerveModuleState getState() {
    double velocity =
        Conversions.falconToMPS(
            mDriveMotor.getRotorVelocityValue(),
            SwerveConstants.Constants.Swerve.wheelCircumference,
            SwerveConstants.Constants.Swerve.driveGearRatio);
    Rotation2d angle =
        Rotation2d.fromDegrees(
            Conversions.falconToDegrees(
                mAngleMotor.getRotorPositionValue(),
                SwerveConstants.Constants.Swerve.angleGearRatio));

    return new SwerveModuleState(velocity, angle);
  }

  /**
   * @return drive position, drive velocity, angle position, angle velocity
   */
  public BaseStatusSignalValue[] getBaseSignals() {
    return new com.ctre.phoenixpro.StatusSignalValue[] {
      mDriveMotor.getRotorPosition(),
      mDriveMotor.getRotorVelocity(),
      mAngleMotor.getRotorPosition(),
      mAngleMotor.getRotorVelocity()
    };
  }
}
