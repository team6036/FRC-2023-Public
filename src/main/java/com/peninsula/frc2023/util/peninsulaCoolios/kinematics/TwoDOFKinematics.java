package com.peninsula.frc2023.util.peninsulaCoolios.kinematics;

import edu.wpi.first.math.geometry.*;
import java.util.Optional;

public class TwoDOFKinematics {
  public double link_1_length_meters, link_2_length_meters;

  public TwoDOFKinematics(double link_1_length_meters, double link_2_length_meters) {
    this.link_1_length_meters = link_1_length_meters;
    this.link_2_length_meters = link_2_length_meters;
  }

  public double[] jointDotsGivenEEDot(Translation2d ee_dot, ArmConfiguration configuration) {
    double x =
        link_1_length_meters * configuration.getAngle1().getCos()
            + link_2_length_meters
            + configuration.getAngle1().plus(configuration.getAngle2()).getCos();
    double y =
        link_1_length_meters * configuration.getAngle1().getSin()
            + link_2_length_meters
            + configuration.getAngle1().plus(configuration.getAngle2()).getSin();

    double x_new = x + ee_dot.getX() * 0.02;
    double y_new = y + ee_dot.getY() * 0.02;

    Optional<ArmConfiguration> moved =
        inverseKinematics(new Pose2d(x_new, y_new, new Rotation2d()));

    if (moved.isEmpty()) return new double[] {0, 0};

    return new double[] {
      (moved.get().getAngle1().getRadians() - configuration.getAngle1().getRadians()) / 0.02,
      (moved.get().getAngle2().getRadians() - configuration.getAngle2().getRadians()) / 0.02
    };
  }

  public Translation2d eeDotGivenJointDots(TwoDOFTorques.State arm) {
    double new_j1 = arm.t1 + arm.t1_dot * 0.02;
    double new_j2 = arm.t2 + arm.t2_dot * 0.02;

    double x =
        link_1_length_meters * Math.cos(arm.t1) + link_2_length_meters + Math.cos(arm.t2 + arm.t1);
    double y =
        link_1_length_meters * Math.sin(arm.t1) + link_2_length_meters + Math.sin(arm.t2 + arm.t1);

    double x_new =
        link_1_length_meters * Math.cos(new_j1) + link_2_length_meters + Math.cos(new_j1 + new_j2);
    double y_new =
        link_1_length_meters * Math.sin(new_j1) + link_2_length_meters + Math.sin(new_j1 + new_j2);

    return new Translation2d((x_new - x) / 0.02, (y_new - y) / 0.02);
  }

  public Transform3d forwardKinematics(ArmConfiguration configuration) {
    Transform3d base_to_joint2 =
        new Transform3d(
            new Translation3d(
                link_1_length_meters,
                new Rotation3d(0, -configuration.getAngle1().getRadians(), 0)),
            new Rotation3d(0, -configuration.getAngle1().getRadians(), 0));
    // Translation defines movement from base flat plane
    // Rotation is the new orientation of the end

    Transform3d joint2_to_end =
        new Transform3d(
            new Translation3d(
                link_2_length_meters,
                new Rotation3d(0, -configuration.getAngle2().getRadians(), 0)),
            new Rotation3d(0, -configuration.getAngle2().getRadians(), 0));

    return base_to_joint2.plus(joint2_to_end);
  }

  public Pose3d joint1Pose(
      ArmConfiguration configuration, Pose2d chassis_pose2d, Transform3d chassis_to_base) {
    Transform3d base_to_joint2 =
        new Transform3d(
            new Translation3d(
                link_1_length_meters,
                new Rotation3d(0, -configuration.getAngle1().getRadians(), 0)),
            new Rotation3d(0, -configuration.getAngle1().getRadians(), 0));
    Pose3d chassis_pose3d = new Pose3d(chassis_pose2d);

    return chassis_pose3d.transformBy(chassis_to_base).transformBy(base_to_joint2);
  }

  public Pose3d endEffectorPose(
      ArmConfiguration configuration, Pose2d chassis_pose2d, Transform3d chassis_to_base) {
    Transform3d base_to_arm = forwardKinematics(configuration);
    Pose3d chassis_pose3d = new Pose3d(chassis_pose2d);

    return chassis_pose3d.transformBy(chassis_to_base).transformBy(base_to_arm);
  }

  public Optional<ArmConfiguration> inverseKinematics(Pose2d wantedEndEffectorPose) {
    double x = wantedEndEffectorPose.getX();
    double y = wantedEndEffectorPose.getY();

    boolean flip = x < 0;

    x = Math.abs(x);

    if (x * x + y * y > 4) return Optional.empty();

    double alpha =
        Math.acos(
            -(x * x + y * y - Math.pow(link_1_length_meters, 2) - Math.pow(link_2_length_meters, 2))
                / (2 * link_1_length_meters * link_2_length_meters));

    Rotation2d angle2 = new Rotation2d(alpha).minus(new Rotation2d(Math.PI));

    if (flip) angle2 = new Rotation2d().minus(angle2);

    double phi = Math.asin(link_2_length_meters * Math.sin(alpha) / Math.sqrt(x * x + y * y));

    double phi_complement = Math.atan(y / x);

    Rotation2d angle1 = new Rotation2d(phi).plus(new Rotation2d(phi_complement));

    if (flip) angle1 = new Rotation2d(Math.PI).minus(angle1);

    return Optional.of(new ArmConfiguration(angle1, angle2));
  }

  public Pose2d centerOfGravity(
      ArmConfiguration configuration,
      double cg1Offset,
      double cg2Offset,
      double mass1,
      double mass2) {

    double cg1_x = configuration.getAngle1().getCos() * cg1Offset;
    double cg1_y = configuration.getAngle1().getSin() * cg1Offset;

    Rotation2d fixedAngle2 = configuration.getAngle2().plus(configuration.getAngle1());

    double cg2_x =
        configuration.getAngle1().getCos() * link_1_length_meters
            + fixedAngle2.getCos() * cg2Offset;
    double cg2_y =
        configuration.getAngle1().getSin() * link_1_length_meters
            + fixedAngle2.getSin() * cg2Offset;

    return new Pose2d(
        new Translation2d(
            (cg1_x * mass1 + cg2_x * mass2) / (mass1 + mass2),
            (cg1_y * mass1 + cg2_y * mass2) / (mass1 + mass2)),
        new Rotation2d());
  }

  public double torqueCOnJoint1(
      ArmConfiguration configuration,
      double cg1Offset,
      double cg2Offset,
      double mass1,
      double mass2) {
    Pose2d cg = centerOfGravity(configuration, cg1Offset, cg2Offset, mass1, mass2);

    return Math.cos(Math.atan2(cg.getY(), cg.getX()))
        * 9.81
        * (mass1 + mass2)
        * Math.hypot(cg.getX(), cg.getY());
  }

  public double torqueCOnJoint2(ArmConfiguration configuration, double cg2Offset, double mass2) {
    Rotation2d global_frame_angle = configuration.getAngle2().plus(configuration.getAngle1());

    return global_frame_angle.getCos() * 9.81 * mass2 * cg2Offset;
  }

  public static class ArmConfiguration {
    private Rotation2d j1;
    private Rotation2d j2;

    public ArmConfiguration(Rotation2d j1, Rotation2d j2) {
      this.j1 = j1;
      this.j2 = j2;
    }

    public Rotation2d getAngle1() {
      return j1;
    }

    public Rotation2d getAngle2() {
      return j2;
    }

    public void add1(double degrees) {
      j1 = j1.plus(Rotation2d.fromDegrees(degrees));
    }

    public void add2(double degrees) {
      j2 = j2.plus(Rotation2d.fromDegrees(degrees));
    }

    @Override
    public String toString() {
      return "ArmConfiguration{" + "j1=" + j1 + ", j2=" + j2 + '}';
    }
  }
}
