package com.peninsula.frc2023.config;

import com.ctre.phoenix.led.*;

public class LightingConstants {
  public static int minVoltageToFunction = 7;

  public static int tot = 69;

  /** Aligned * */
  public static final Animation blueSolid = new StrobeAnimation(0, 0, 255, 0, 0.0, tot);
  /** Aligning * */
  public static final Animation blueBlink = new StrobeAnimation(0, 0, 255, 0, 0.1, tot);

  /** Cube aligned * */
  public static final Animation purpleSolid = new StrobeAnimation(148, 0, 211, 0, 0.0, tot);
  /** Cube * */
  public static final Animation purpleBlink = new StrobeAnimation(148, 0, 211, 0, 0.1, tot);

  /** Cone aligned * */
  public static final Animation goldSolid = new StrobeAnimation(179, 99, 19, 0, 0.0, tot);
  /** Cone * */
  public static final Animation goldBlink = new StrobeAnimation(179, 99, 19, 0, 0.1, tot);

  /** Rainbow * */
  public static final Animation rainbow = new RainbowAnimation(1.0, 0.8, tot);
}
