package com.peninsula.frc2023.behavior.routines;

import com.peninsula.frc2023.behavior.TimeoutRoutineBase;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.ReadOnly;
import com.peninsula.frc2023.robot.RobotState;

public class VisionSetOverride extends TimeoutRoutineBase {

  public boolean wantedOff = false;

  public VisionSetOverride(double timeout, boolean off) {
    super(timeout);
    wantedOff = off;
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    //    return Math.abs(state.intakePosRotations - IntakeConstants.downSetpointRot) < 0.1;
    return false;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    state.visionOverrideOff = wantedOff;
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {}
}
