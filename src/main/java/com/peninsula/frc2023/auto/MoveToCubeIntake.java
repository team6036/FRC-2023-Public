package com.peninsula.frc2023.auto;

import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.behavior.SequentialRoutine;
import com.peninsula.frc2023.behavior.routines.IntakeDeploy;
import com.peninsula.frc2023.behavior.routines.arm.RunArmTrajectoryRoutine;
import com.peninsula.frc2023.subsystems.Arm;

public class MoveToCubeIntake implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    return new SequentialRoutine(
        new IntakeDeploy(0.2), new RunArmTrajectoryRoutine(Arm.Positions.INTAKE, 0.7));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
