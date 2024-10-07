package com.peninsula.frc2023.config;

// import com.peninsula.frc2023.vision.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

@SuppressWarnings("java:S1104")
public class VisionConstants {

  public static AprilTagFieldLayout aprilTagFieldLayout;

  public static boolean useMultiTag = true;

  public static double kMaxPoseAmbiguity = 0.2;
  public static double maxDistanceMeters = 3;

  static {
    try {
      aprilTagFieldLayout =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory().toPath() + "/2023-chargedup.json");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static double inch_to_meter = 0.0254;

  public static double meters_front_back = 10.735 * inch_to_meter;
  public static double meters_right_left = 12.129 * inch_to_meter;

  public static double meters_up_down = (8.856) * inch_to_meter;

  public static Rotation3d d = new Rotation3d(new Quaternion(0, 0, 0, 0).toRotationVector());

  public static Transform3d robot_to_camera_NETWORK =
      new Transform3d(
          new Translation3d(meters_front_back, -meters_right_left, meters_up_down),
          new Rotation3d(new Quaternion(0.943, -0.028, -0.083, 0.320).toRotationVector()));

  public static Transform3d robot_to_camera_RIO =
      new Transform3d(
          new Translation3d(meters_front_back, meters_right_left, meters_up_down),
          new Rotation3d(new Quaternion(0.943, 0.028, -0.083, -0.320).toRotationVector()));
}
