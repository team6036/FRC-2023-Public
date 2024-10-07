package com.peninsula.frc2023.subsystems;

import com.ctre.phoenix.led.Animation;
import com.peninsula.frc2023.config.LightingConstants;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.ReadOnly;
import com.peninsula.frc2023.robot.RobotState;
import edu.wpi.first.wpilibj.RobotController;

public class Lighting extends SubsystemBase {

  public enum State {
    OFF,
    IDLE,
    CONE,
    CONE_ALIGNED,
    CUBE,
    CUBE_ALIGNED,
    ALIGNING,
    ALIGNED
  }

  private static Lighting sInstance = new Lighting();
  private State mState;

  private Lighting() {}

  public static Lighting getInstance() {
    return sInstance;
  }

  public State getState() {
    return mState;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    State wantedState = commands.wantedLighting;

    if (RobotController.getBatteryVoltage() < LightingConstants.minVoltageToFunction)
      wantedState = State.OFF;

    boolean isNewState = mState != wantedState;

    mState = wantedState;
    if (isNewState) {
      switch (mState) {
        case IDLE:
          mOutput = LightingConstants.rainbow;
          break;
        case CONE:
          mOutput = LightingConstants.goldSolid;
          break;
        case CONE_ALIGNED:
          mOutput = LightingConstants.goldBlink;
          break;
        case CUBE:
          mOutput = LightingConstants.purpleSolid;
          break;
        case CUBE_ALIGNED:
          mOutput = LightingConstants.purpleBlink;
          break;
        case ALIGNING:
          mOutput = LightingConstants.blueBlink;
          break;
        case ALIGNED:
          mOutput = LightingConstants.blueSolid;
          break;
      }
    }
  }

  Animation mOutput;

  public Animation getOutput() {
    return mOutput;
  }
}
