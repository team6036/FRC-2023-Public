package com.peninsula.frc2023.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

public class LoopOverrunDebugger {

  private static class Measurement {

    String name;
    double durationSeconds;

    Measurement(String name, double durationSeconds) {
      this.name = name;
      this.durationSeconds = durationSeconds;
    }
  }

  public Timer mTimer = new Timer();
  private final ArrayList<Measurement> mMeasurements = new ArrayList<>(8);

  public LoopOverrunDebugger(String name, double printDurationSeconds) {
    this(name);
  }

  public LoopOverrunDebugger(String name) {
    mTimer.start();
  }

  public void addPoint(String name) {
    mMeasurements.add(new Measurement(name, mTimer.get()));
  }

  public void finish() {
    SmartDashboard.putNumber("loop time ms", mTimer.get() * 1000.0);
    printSummary();
  }

  private void printSummary() {
    mMeasurements.add(0, new Measurement("Start", 0));
    for (int i = 1; i < mMeasurements.size(); ++i) {
      SmartDashboard.putNumber(
          mMeasurements.get(i).name,
          (mMeasurements.get(i).durationSeconds - mMeasurements.get(i - 1).durationSeconds) * 1000);
    }
  }

  public void reset() {
    mTimer.reset();
    mMeasurements.clear();
  }
}
