package com.peninsula.frc2023.auto;

import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.behavior.SequentialRoutine;
import com.peninsula.frc2023.behavior.routines.IntakeRetract;
import com.peninsula.frc2023.behavior.routines.arm.RunArmTrajectoryRoutine;
import com.peninsula.frc2023.subsystems.Arm;

public class MoveBackFromIntake implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    return new SequentialRoutine(
        new RunArmTrajectoryRoutine(Arm.Positions.STOWED, 0.7), new IntakeRetract(0.5));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
