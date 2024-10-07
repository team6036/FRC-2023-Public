package com.peninsula.frc2023.auto;

import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.util.Util;

public interface AutoBase {

  RoutineBase getRoutine();

  default String getName() {
    return Util.classToJsonName(getClass());
  }
}
