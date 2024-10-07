package com.peninsula.frc2023.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenixpro.StatusSignalValue;
import com.kauailabs.navx.frc.AHRS;
import com.peninsula.frc2023.config.PortConstants;
import com.peninsula.frc2023.config.SwerveConstants;
import com.peninsula.frc2023.util.MacroPad;
import com.peninsula.frc2023.util.config.FalconFactoryPro;
import com.peninsula.frc2023.util.control.FalconPro;
import com.peninsula.frc2023.util.swerveDrivers.SwerveModule;
import edu.wpi.first.wpilibj.*;
import org.photonvision.PhotonCamera;

/**
 * Represents all hardware components of the robot. Singleton class. Should only be used in robot
 * package. Subdivides hardware into subsystems.
 */
public class HardwareAdapter {
  static class IntakeHardware {
    private static IntakeHardware sInstance;
    final FalconPro rollerMotor;
    final FalconPro actuationMotor;

    final StatusSignalValue<Double> getPosition;

    private IntakeHardware() {
      rollerMotor =
          FalconFactoryPro.createDefaultFalconPro(
              PortConstants.intakeRollerMotor, "IntakeRollerMotor");
      actuationMotor =
          FalconFactoryPro.createDefaultFalconPro(
              PortConstants.intakeActuationMotor, "IntakeActuationMotor");

      getPosition = actuationMotor.getPosition();
    }

    static IntakeHardware getInstance() {
      if (sInstance == null) sInstance = new IntakeHardware();
      return sInstance;
    }
  }

  /**
   * 4 Falcon 500s (controlled by Talon FX), 1 Pigeon IMU Gyro connected via Talon SRX data cable.
   */
  public static class SwerveHardware {

    private static SwerveHardware sInstance;

    // CW Starting from top left module
    // <Drive Motor, Turn Motor, Encoder>
    public final SwerveModule FL, FR, BL, BR;
    public final SwerveModule[] modules;

    public final AHRS gyro;

    private SwerveHardware() {

      FL = new SwerveModule(0, SwerveConstants.Constants.Swerve.Mod0.constants);
      FR = new SwerveModule(1, SwerveConstants.Constants.Swerve.Mod1.constants);
      BL = new SwerveModule(2, SwerveConstants.Constants.Swerve.Mod2.constants);
      BR = new SwerveModule(3, SwerveConstants.Constants.Swerve.Mod3.constants);
      modules = new SwerveModule[] {FL, FR, BL, BR};

      gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
    }

    static SwerveHardware getInstance() {
      if (sInstance == null) sInstance = new SwerveHardware();
      return sInstance;
    }
  }

  static class ArmHardware {
    private static ArmHardware sInstance;

    final FalconPro joint1_ceo, joint2_ceo;
    final FalconPro joint1_employee, joint2_employee;

    private ArmHardware() {

      joint1_ceo = FalconFactoryPro.createDefaultFalconPro(PortConstants.armJoint1, "joint1_c");
      joint2_ceo = FalconFactoryPro.createDefaultFalconPro(PortConstants.armJoint2, "joint2_c");

      joint1_employee =
          FalconFactoryPro.createDefaultFalconPro(PortConstants.armJoint1_2, "joint1_e");
      joint2_employee =
          FalconFactoryPro.createDefaultFalconPro(PortConstants.armJoint2_2, "joint2_e");
    }

    static ArmHardware getInstance() {
      if (sInstance == null) sInstance = new ArmHardware();
      return sInstance;
    }
  }

  static class VisionHardware {

    private static VisionHardware sInstance;

    PhotonCamera cameraRIO;
    PhotonCamera cameraNETWORK;

    private VisionHardware() {
      cameraRIO = new PhotonCamera("002-3");
      cameraNETWORK = new PhotonCamera("002-1");
    }

    static VisionHardware getInstance() {
      if (sInstance == null) sInstance = new VisionHardware();
      return sInstance;
    }
  }

  /** 22 Xbox Controller */
  static class JoystickHardware {

    private static JoystickHardware sInstance;

    final XboxController driverXboxController;
    final XboxController operatorXboxController;
    final MacroPad board;

    private JoystickHardware() {
      driverXboxController = new XboxController(PortConstants.kDriverId);
      operatorXboxController = new XboxController(PortConstants.kOperatorId);
      board = new MacroPad();
    }

    static JoystickHardware getInstance() {
      if (sInstance == null) sInstance = new JoystickHardware();
      return sInstance;
    }
  }

  static class GripperHardware {
    private static GripperHardware sInstance;

    final FalconPro falcon;
    final StatusSignalValue<Double> getCurrent;
    final StatusSignalValue<Double> getVelo;

    final DigitalInput irSensor;

    private GripperHardware() {
      falcon = FalconFactoryPro.createDefaultFalconPro(PortConstants.gripper_id, "gripper");
      irSensor = new DigitalInput(PortConstants.ir_id);

      getCurrent = falcon.getTorqueCurrent();
      getVelo = falcon.getVelocity();
    }

    static GripperHardware getInstance() {
      if (sInstance == null) sInstance = new GripperHardware();
      return sInstance;
    }
  }

  static class LightingHardware {
    private static LightingHardware sInstance;

    final CANdle candle;

    private LightingHardware() {
      candle = new CANdle(PortConstants.kLighting, "swerve");
    }

    static LightingHardware getInstance() {
      if (sInstance == null) sInstance = new LightingHardware();
      return sInstance;
    }
  }

  private HardwareAdapter() {}
}
