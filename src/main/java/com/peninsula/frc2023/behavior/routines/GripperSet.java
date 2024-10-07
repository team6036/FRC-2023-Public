package com.peninsula.frc2023.behavior.routines;

import com.peninsula.frc2023.behavior.TimeoutRoutineBase;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.ReadOnly;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.subsystems.Gripper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GripperSet extends TimeoutRoutineBase {

  public Gripper.State wantedState;

  public GripperSet(Gripper.State state, double timeout) {
    super(timeout);
    this.wantedState = state;
    SmartDashboard.putBoolean("Want on", state == Gripper.State.ON);
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.gripperWanted = wantedState;
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {}
}
