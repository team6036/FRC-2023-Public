package com.peninsula.frc2023.subsystems;

import com.peninsula.frc2023.config.IntakeConstants;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.ReadOnly;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.util.control.ControllerOutput;
import com.peninsula.frc2023.util.control.ProfiledGains;

public class Intake extends SubsystemBase {

  public enum State {
    ON,
    MOVE,
    OFF
  }

  private final ControllerOutput mRollerOutputs = new ControllerOutput();
  private final ControllerOutput mActuationOutputs = new ControllerOutput();
  private static final Intake sInstance = new Intake();

  private Intake() {}

  public static Intake getInstance() {
    return sInstance;
  }

  public ControllerOutput getRollerOutputs() {
    return mRollerOutputs;
  }

  public ControllerOutput getActuationOutputs() {
    return mActuationOutputs;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    switch (commands.intakeWanted) {
      case ON:
        mRollerOutputs.setVolt(IntakeConstants.intakeRunVolts);
        mActuationOutputs.setTargetPositionProfiled(
            IntakeConstants.downSetpointRot, new ProfiledGains());
        break;
      case OFF:
        mRollerOutputs.setIdle();
        mActuationOutputs.setTargetPositionProfiled(
            IntakeConstants.stowedSetpoint, new ProfiledGains());
        break;
    }
  }
}
