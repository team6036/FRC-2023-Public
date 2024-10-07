package com.peninsula.frc2023.robot;

import com.peninsula.frc2023.config.IntakeConstants;
import com.peninsula.frc2023.robot.HardwareAdapter.JoystickHardware;
import com.peninsula.frc2023.subsystems.*;
import com.peninsula.frc2023.util.Util;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFKinematics;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFTorques;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Set;

public class HardwareReader {

  private static final String kLoggerTag = Util.classToJsonName(HardwareReader.class);

  public HardwareReader() {}

  /**
   * Takes all of the sensor data from the hardware, and unwraps it into the current {@link
   * RobotState}.
   */
  void readState(Set<SubsystemBase> enabledSubsystems, RobotState state) {
    Robot.sLoopDebugger.reset();
    readGameAndFieldState(state);
    Robot.sLoopDebugger.addPoint("Read game");
    if (state.gamePeriod == RobotState.GamePeriod.TELEOP) readJoystickState(state);
    Robot.sLoopDebugger.addPoint("Read joystick");
    if (enabledSubsystems.contains(Arm.getInstance())) readArmState(state);
    Robot.sLoopDebugger.addPoint("Read arm");
    if (enabledSubsystems.contains(Gripper.getInstance())) readGripperState(state);
    Robot.sLoopDebugger.addPoint("Read gripper");
    if (enabledSubsystems.contains(Intake.getInstance())) readIntakeState(state);
    Robot.sLoopDebugger.addPoint("Read intake");
    Robot.sLoopDebugger.finish();
  }

  private void readGameAndFieldState(RobotState state) {
    state.gameData = DriverStation.getGameSpecificMessage();
    state.cycles += 1;
    state.gameTimeS = Timer.getFPGATimestamp();
  }

  private void readGripperState(RobotState state) {
    var hardware = HardwareAdapter.GripperHardware.getInstance();

    state.currentTorqueGripper = hardware.getCurrent.refresh().getValue();
    state.gripperVelo = hardware.getVelo.refresh().getValue();

    state.ir_reading = !hardware.irSensor.get(); // Check if ! or nothing
  }

  private void readJoystickState(RobotState state) {
    var hardware = JoystickHardware.getInstance();

    state.driverLeftX = Util.handleDeadBand(hardware.driverXboxController.getLeftX(), 0.09);
    state.driverLeftY = Util.handleDeadBand(hardware.driverXboxController.getLeftY(), 0.09);
    state.driverRightY = Util.handleDeadBand(hardware.driverXboxController.getRightY(), 0.09);
    state.driverRightX = Util.handleDeadBand(hardware.driverXboxController.getRightX(), 0.04);
    state.driverRt = hardware.driverXboxController.getRightTriggerAxis();
    state.operatorAPressed = hardware.operatorXboxController.getAButton();
    state.operatorRtPressed = hardware.operatorXboxController.getRightTriggerAxis() > 0.1;
    state.operatorLtPressed = hardware.operatorXboxController.getLeftTriggerAxis() > 0.5;
    state.operatorRbPressed = hardware.operatorXboxController.getRightBumper();
    state.operatorLbPressed = hardware.operatorXboxController.getLeftBumper();
    state.operatorDPadLeftPressed = hardware.operatorXboxController.getLeftStickButton();
    state.operatorDPadRightPressed = hardware.operatorXboxController.getRightStickButton();
    state.driverAPressed = hardware.driverXboxController.getAButton();
    state.driverBPressed = hardware.driverXboxController.getBButtonPressed();
    state.driverRbPressed = hardware.driverXboxController.getRightBumper();
    state.operatorLeftX = Util.handleDeadBand(hardware.operatorXboxController.getLeftX(), 0.09);
    state.operatorLeftY = Util.handleDeadBand(hardware.operatorXboxController.getLeftY(), 0.09);
    state.operatorRightX = Util.handleDeadBand(hardware.operatorXboxController.getRightX(), 0.2);
    state.operatorRightY = Util.handleDeadBand(hardware.operatorXboxController.getRightY(), 0.2);
    state.operatorAPressed = hardware.operatorXboxController.getAButton();
    state.operatorBPressed = hardware.operatorXboxController.getBButton();
    state.operatorXPressed = hardware.operatorXboxController.getXButton();
    state.operatorYPressed = hardware.operatorXboxController.getYButton();
    state.driverLtPressed = hardware.driverXboxController.getLeftTriggerAxis() > 0.5;

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 9; j++) {
        state.pressedGrid[i][j] = hardware.board.RC(i, j);
      }
    }
    state.grip = hardware.board.grip();
    state.stow = hardware.board.stow();
    state.cone = hardware.board.conePick();
    state.cube = hardware.board.cubePick();
    state.double_sub = hardware.board.doublePick();
    state.intake = hardware.board.intake();
  }

  private void readArmState(RobotState state) {
    var hardware = HardwareAdapter.ArmHardware.getInstance();

    if (!Robot.isRobotReal()) {
      state.armState =
          new TwoDOFKinematics.ArmConfiguration(Arm.getInstance().want1, Arm.getInstance().want2);
    } else {
      double aPos = hardware.joint1_ceo.getRotorPositionValue();
      double aVel = hardware.joint1_ceo.getRotorVelocityValue();
      Rotation2d a = Arm.getInstance().motorToJoint1Angle(aPos);
      Rotation2d b =
          Arm.getInstance()
              .motorToJoint2FrameAngle(aPos, hardware.joint2_ceo.getRotorPositionValue());

      Rotation2d j1velo = Arm.getInstance().motorToJoint1Angle(aVel);
      Rotation2d j2velo =
          Arm.getInstance()
              .motorToJoint2FrameAngle(aVel, hardware.joint2_ceo.getRotorVelocityValue());

      state.armState = new TwoDOFKinematics.ArmConfiguration(a, b);
      state.armQ =
          new TwoDOFTorques.State(
              a.getRadians(), b.getRadians(), j1velo.getRadians(), j2velo.getRadians());
    }
  }

  private void readVisionState(RobotState state) {
    var hardware = HardwareAdapter.VisionHardware.getInstance();

    state.photonPipelineResultRIO = hardware.cameraRIO.getLatestResult();
    state.photonPipelineResultNETWORK = hardware.cameraNETWORK.getLatestResult();

    RobotStateEstimator.getInstance().updateVisionPoseEstimator(state);
  }

  private void readIntakeState(RobotState state) {
    var hardware = HardwareAdapter.IntakeHardware.getInstance().actuationMotor;

    state.intakePosRotations = hardware.getRotorPositionValue() / IntakeConstants.gearRatio;

    SmartDashboard.putNumber(
        "downSet", IntakeConstants.downSetpointRot / IntakeConstants.gearRatio);
    SmartDashboard.putNumber("stowSet", IntakeConstants.stowedSetpoint / IntakeConstants.gearRatio);
    SmartDashboard.putNumber("posIntake", state.intakePosRotations);
    SmartDashboard.putString("intake state read", state.intakeState.name());

    if (Math.abs(
            state.intakePosRotations - IntakeConstants.downSetpointRot / IntakeConstants.gearRatio)
        < 0.05) {
      state.intakeState = Intake.State.ON;
    } else if (Math.abs(
            state.intakePosRotations - IntakeConstants.stowedSetpoint / IntakeConstants.gearRatio)
        < 0.05) {
      state.intakeState = Intake.State.OFF;
    } else {
      state.intakeState = Intake.State.MOVE;
    }
  }
}
