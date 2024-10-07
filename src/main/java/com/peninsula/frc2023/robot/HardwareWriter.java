package com.peninsula.frc2023.robot;

import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.peninsula.frc2023.config.IntakeConstants;
import com.peninsula.frc2023.subsystems.*;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFKinematics;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Set;

public class HardwareWriter {

  public static final int kTimeoutMs = 150, kPidIndex = 0;

  void configureHardware(Set<SubsystemBase> enabledSubsystems) {
    if (enabledSubsystems.contains(Swerve.getInstance())) configureSwerveHardware();
    if (enabledSubsystems.contains(Vision.getInstance())) configureVisionHardware();
    if (enabledSubsystems.contains(Arm.getInstance())) configureArmHardware();
    if (enabledSubsystems.contains(Gripper.getInstance())) configureGripperHardware();
    if (enabledSubsystems.contains(Intake.getInstance())) configureIntakeHardware();
  }

  private void configureArmHardware() {
    var hardware = HardwareAdapter.ArmHardware.getInstance();

    MotorOutputConfigs m = new MotorOutputConfigs();
    m.NeutralMode = NeutralModeValue.Brake;

    hardware.joint1_ceo.getConfigurator().apply(m);
    hardware.joint1_employee.getConfigurator().apply(m);

    hardware.joint2_ceo.getConfigurator().apply(m);
    hardware.joint2_employee.getConfigurator().apply(m);

    hardware.joint1_ceo.setInverted(false);
    hardware.joint1_employee.setInverted(false);

    hardware.joint2_ceo.setInverted(true);
    hardware.joint2_employee.setInverted(true);

    hardware.joint1_ceo.setRotorPosition(
        Arm.getInstance().joint1AngleToMotor(new Rotation2d(3.1415 / 2)), 0.05);
    hardware.joint1_employee.setRotorPosition(
        Arm.getInstance().joint1AngleToMotor(new Rotation2d(3.1415 / 2)), 0.05);

    hardware.joint2_ceo.setRotorPosition(
        Arm.getInstance().joint2AngleToMotor(new Rotation2d(3.1415 / 2), new Rotation2d(-3.1415)),
        0.05);
    hardware.joint2_employee.setRotorPosition(
        Arm.getInstance().joint2AngleToMotor(new Rotation2d(3.1415 / 2), new Rotation2d(-3.1415)),
        0.05);
  }

  private void configureGripperHardware() {
    var hardware = HardwareAdapter.GripperHardware.getInstance();
    hardware.falcon.setInverted(true);
  }

  private void configureIntakeHardware() {
    var hardware = HardwareAdapter.IntakeHardware.getInstance();

    Slot0Configs actuationConfigs = new Slot0Configs();

    actuationConfigs.kP = IntakeConstants.intakeGains.p;
    actuationConfigs.kI = IntakeConstants.intakeGains.i;
    actuationConfigs.kD = IntakeConstants.intakeGains.d;

    MotionMagicConfigs actuationMotionConfigs = new MotionMagicConfigs();

    actuationMotionConfigs.MotionMagicCruiseVelocity = IntakeConstants.maxSpeedRotPS;
    actuationMotionConfigs.MotionMagicAcceleration = IntakeConstants.maxAccelRotPS2;

    hardware.actuationMotor.getConfigurator().apply(actuationConfigs);
    hardware.actuationMotor.getConfigurator().apply(actuationMotionConfigs);

    hardware.actuationMotor.setInverted(true);
    hardware.rollerMotor.setInverted(true);

    hardware.actuationMotor.setRotorPosition(0);
  }

  private void configureSwerveHardware() {}

  private void configureVisionHardware() {}

  /** Updates the hardware to run with output values of {@link SubsystemBase}'s. */
  void writeHardware(Set<SubsystemBase> enabledSubsystems, @ReadOnly RobotState robotState) {
    if (enabledSubsystems.contains(Swerve.getInstance())) updateSwerve(robotState);
    if (enabledSubsystems.contains(Arm.getInstance())) updateArm(robotState);
    if (enabledSubsystems.contains(Gripper.getInstance())) updateGripper();
    if (enabledSubsystems.contains(Lighting.getInstance())) updateLighting(robotState);
    if (enabledSubsystems.contains(Intake.getInstance())) updateIntake();
    updateJoysticks();
  }

  private void updateIntake() {
    var hardware = HardwareAdapter.IntakeHardware.getInstance();

    var outputsRoller = Intake.getInstance().getRollerOutputs();
    var outputsActuation = Intake.getInstance().getActuationOutputs();

    hardware.actuationMotor.setOutput(outputsActuation, false);
    hardware.rollerMotor.setOutput(outputsRoller, false);
  }

  private void updateGripper() {
    var hardware = HardwareAdapter.GripperHardware.getInstance();
    var outputs = Gripper.getInstance().getOutputs();

    hardware.falcon.setOutput(outputs, false);
  }

  private void updateArm(RobotState state) {
    var hardware = HardwareAdapter.ArmHardware.getInstance();

    if (Robot.isRobotReal()) {
      hardware.joint1_ceo.setOutput(Arm.getInstance().getJoint1Output(), false);
      hardware.joint1_employee.setOutput(Arm.getInstance().getJoint1Output(), false);

      hardware.joint2_ceo.setOutput(Arm.getInstance().getJoint2Output(), false);
      hardware.joint2_employee.setOutput(Arm.getInstance().getJoint2Output(), false);
    }

    if (!Robot.isRobotReal()) {
      state.armState =
          new TwoDOFKinematics.ArmConfiguration(Arm.getInstance().want1, Arm.getInstance().want2);
    }
  }

  private void updateSwerve(RobotState state) {
    var hardware = HardwareAdapter.SwerveHardware.getInstance();
    var outputs = Swerve.getInstance().getOutputs();

    if (Swerve.getInstance().getZero()) {
      hardware.gyro.zeroYaw();
    }

    if (!outputs.isIdle()) {
      hardware.FL.setDesiredState(
          outputs.getStates()[0],
          state.realModuleStates[0],
          false); // TODO, implement swerve optimization in OutputGenerator
      hardware.FR.setDesiredState(outputs.getStates()[1], state.realModuleStates[1], false);
      hardware.BL.setDesiredState(outputs.getStates()[2], state.realModuleStates[2], false);
      hardware.BR.setDesiredState(outputs.getStates()[3], state.realModuleStates[3], false);
    }
  }

  private void updateLighting(RobotState state) {
    var hardware = HardwareAdapter.LightingHardware.getInstance();
    if (Lighting.getInstance().getState() != state.lastSet) {
      hardware.candle.animate(Lighting.getInstance().getOutput());
      state.lastSet = Lighting.getInstance().getState();
    }
  }

  private void updateJoysticks() {}
}
