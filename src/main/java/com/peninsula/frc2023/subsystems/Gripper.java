package com.peninsula.frc2023.subsystems;

import com.peninsula.frc2023.config.GripperConstants;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.util.control.ControllerOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gripper extends SubsystemBase {

  public enum State {
    ON,
    OFF,
    SPIT
  }

  public static Gripper sInstance = new Gripper();
  public State mState = State.ON;
  public ControllerOutput mOutputs = new ControllerOutput();

  public static Gripper getInstance() {
    return sInstance;
  }

  public ControllerOutput getOutputs() {
    return mOutputs;
  }

  @Override
  public void update(Commands commands, RobotState state) {
    State wanted = commands.gripperWanted;

    if (mState != wanted) {
      mState = wanted;
    }

    SmartDashboard.putNumber("GV", state.gripperVelo);
    SmartDashboard.putString("mState", mState.toString());
    SmartDashboard.putString("gripperWanted", wanted.toString());
    SmartDashboard.putBoolean("irDetected", state.ir_reading);
    switch (mState) {
      case ON:
        if (state.ir_reading) {
          mOutputs.setPercentOutput(GripperConstants.heldSpeed);
        } else {
          mOutputs.setPercentOutput(GripperConstants.runSpeed);
        }
        break;
      case OFF:
        mOutputs.setPercentOutput(GripperConstants.stopSpeed);
        break;
      case SPIT:
        mOutputs.setPercentOutput(GripperConstants.stopSpeed * 5);
        break;
    }
  }
}
