package com.peninsula.frc2023.behavior.routines;

import com.peninsula.frc2023.behavior.TimeoutRoutineBase;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.ReadOnly;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.subsystems.Intake;

public class IntakeRetract extends TimeoutRoutineBase {

  public Intake.State wantedState;

  public IntakeRetract(double timeout) {
    super(timeout);
    this.wantedState = Intake.State.OFF;
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    //    return Math.abs(state.intakePosRotations - IntakeConstants.stowedSetpoint) < 0.1;
    return false;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.intakeWanted = wantedState;
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {}
}