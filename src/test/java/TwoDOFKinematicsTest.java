import com.peninsula.frc2023.config.ArmConstants;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFKinematics;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFKinematics.ArmConfiguration;
import edu.wpi.first.math.geometry.*;
import java.util.Optional;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class TwoDOFKinematicsTest {

  double epsilon = 1E-3;

  @Test
  public void testFull() {
    ArmConfiguration config =
        new ArmConfiguration(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(-90));

    Pose2d chassis_pose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(180));

    TwoDOFKinematics kinematics = new TwoDOFKinematics(1, 1);

    Pose3d ee_pose3d = kinematics.endEffectorPose(config, chassis_pose2d, new Transform3d());

    Assert.assertEquals(ee_pose3d.getX(), -1, epsilon);
    Assert.assertEquals(ee_pose3d.getZ(), 1, epsilon);
    Assert.assertEquals(ee_pose3d.getRotation().getZ(), Math.PI, epsilon);

    System.out.println(ee_pose3d);
  }

  @Test
  public void inverseTest() {
    TwoDOFKinematics kinematics = new TwoDOFKinematics(1, 1);

    for (int i = 0; i < 100; i++)
      test(kinematics, Math.random() * 1.8 - 0.9, Math.random() * 1.8 - 0.9);
  }

  @Test
  public void tt() {
    Optional<ArmConfiguration> a =
        ArmConstants.dof.inverseKinematics(
            new Pose2d(new Translation2d(0.70, 1.4), new Rotation2d()));
    System.out.println(a);
    System.out.println(a.get().getAngle1() + " " + a.get().getAngle2());
  }

  public void test(TwoDOFKinematics kinematics, double x, double y) {
    Optional<ArmConfiguration> configOptional =
        kinematics.inverseKinematics(new Pose2d(new Translation2d(x, y), new Rotation2d()));

    Assert.assertTrue(configOptional.isPresent());

    ArmConfiguration configuration = configOptional.get();

    Transform3d forward = kinematics.forwardKinematics(configuration);

    Assert.assertEquals(x, forward.getX(), epsilon);
    Assert.assertEquals(y, forward.getZ(), epsilon);
  }

  @Test
  public void torque1() {
    TwoDOFKinematics dof = ArmConstants.dof;

    ArmConfiguration config =
        new ArmConfiguration(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(0));

    Assert.assertEquals(
        0,
        dof.torqueCOnJoint1(
            config,
            ArmConstants.cg1Offset,
            ArmConstants.cg2Offset,
            ArmConstants.mass1,
            ArmConstants.mass2),
        epsilon);
  }

  @Test
  public void torque2() {
    TwoDOFKinematics dof = ArmConstants.dof;

    ArmConfiguration config =
        new ArmConfiguration(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90));

    Assert.assertEquals(
        0, dof.torqueCOnJoint2(config, ArmConstants.cg2Offset, ArmConstants.mass2), epsilon);
  }
}
