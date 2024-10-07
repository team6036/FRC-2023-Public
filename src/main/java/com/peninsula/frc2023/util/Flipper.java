package com.peninsula.frc2023.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Flipper {
  public static double FL = 16.54175;

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(
        new Translation2d(FL - pose.getX(), pose.getY()), flipRot(pose.getRotation()));
  }

  public static Rotation2d flipRot(Rotation2d rot) {
    return new Rotation2d(-rot.getCos(), rot.getSin());
  }
}
