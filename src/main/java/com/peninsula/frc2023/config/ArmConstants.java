package com.peninsula.frc2023.config;

import com.peninsula.frc2023.util.control.Gains;
import com.peninsula.frc2023.util.peninsulaCoolios.PathHolder;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFKinematics;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFTorques;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;

public class ArmConstants {
  public static double l1 = 0.9144;
  public static double l2 = 0.7578;
  public static TwoDOFKinematics dof = new TwoDOFKinematics(l1, l2); // 31in and 29in

  public static Transform3d chassis_to_base =
      new Transform3d(new Translation3d(0, 0.2617, 0.31785), new Rotation3d(0, 0, 0));

  private static final double ozin2_to_kgm2 = 1.829e-5;

  /* Physical Constants */
  public static double mass1 = 2.84048047, mass2 = 6.484471473; // kg
  public static double mass2_with_cone = 7.100790106; // kg

  public static double cg1Offset = 0.38735, cg2Offset = 0.2917698; // m
  public static double cg2Offset_with_cone = 0.323469; // m

  public static double joint1GearRatio = (60.0 / 8) * (48.0 / 18) * (36.0 / 12); // Reduction

  public static double joint2MotorToAxel = (56.0 / 14) * (60.0 / 26) * (36.0 / 12); // Reduction
  public static double joint2AxelToJoint = (36.0 / 24); // Reduction

  public static double momentJ1 = 22159.905 * ozin2_to_kgm2;
  public static double momentJ2 = 31780.362 * ozin2_to_kgm2;
  public static double momentJ2_with_cone = 36276.136 * ozin2_to_kgm2;

  public static double motorsJ1 = 2;
  public static double motorsJ2 = 2;

  public static TwoDOFTorques torquesFF =
      new TwoDOFTorques(
          dof.link_1_length_meters, cg1Offset, cg2Offset, mass1, mass2, momentJ1, momentJ2);

  public static TwoDOFTorques torquesFF_with_cone =
      new TwoDOFTorques(
          dof.link_1_length_meters,
          cg1Offset,
          cg2Offset_with_cone,
          mass1,
          mass2_with_cone,
          momentJ1,
          momentJ2_with_cone);

  /* Gains */
  public static Gains joint1MK2Gains = new Gains(7, 0, 0, 0, 0, 0);
  public static Gains joint2MK2Gains = new Gains(8, 0, 0.8, 0, 0, 0);

  public static Gains jointOld1Gains = new Gains(0.7, 0, 0.02, 0, 0, 0);
  public static Gains jointOld2Gains = new Gains(1.5, 0, 0.55, 0, 0, 0);

  public static double antiGravFeedforwardJ1Coeff = 0.00141;
  public static double antiGravFeedforwardJ2Coeff = 0.00481;

  public static double torqueOnJ1(TwoDOFKinematics.ArmConfiguration config) {
    return dof.torqueCOnJoint1(config, cg1Offset, cg2Offset, mass1, mass2);
  }

  public static double torqueOnJ2(TwoDOFKinematics.ArmConfiguration config) {
    return dof.torqueCOnJoint2(config, cg2Offset, mass2);
  }

  /* Pathing */
  public static PathHolder paths;

  static {
    try {
      paths = new PathHolder();
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
