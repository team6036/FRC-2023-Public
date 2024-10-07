package com.peninsula.frc2023.subsystems;

import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.ReadOnly;
import com.peninsula.frc2023.robot.RobotState;
import org.photonvision.common.hardware.VisionLEDMode;

/** http://gloworm.local:5800/#/dashboard */
public class Vision extends SubsystemBase {

  public enum State {
    ON,
    OFF
  }

  public class VisionOutputs {

    int pipelineIndex;
    VisionLEDMode ledMode;
  }

  private VisionOutputs mOutputs = new VisionOutputs();
  private static Vision sInstance = new Vision();
  private State lastVisionWanted = null;

  private Vision() {}

  public static Vision getInstance() {
    return sInstance;
  }

  public VisionOutputs getOutputs() {
    return mOutputs;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    mOutputs.pipelineIndex = 0;

    if (lastVisionWanted != commands.visionWanted) {
      switch (commands.visionWanted) {
        case ON:
          mOutputs.ledMode = VisionLEDMode.kOn;
        case OFF:
          mOutputs.ledMode = VisionLEDMode.kOff;
      }
    }
    lastVisionWanted = commands.visionWanted;
  }
}
