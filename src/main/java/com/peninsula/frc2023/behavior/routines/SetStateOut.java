package com.peninsula.frc2023.behavior.routines;

import com.peninsula.frc2023.behavior.TimeoutRoutineBase;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.ReadOnly;
import com.peninsula.frc2023.robot.RobotState;

public class SetStateOut extends TimeoutRoutineBase {

  public SetStateOut(double timeout) {
    super(timeout);
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    state.intakeDeployed = true;
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {}
}
