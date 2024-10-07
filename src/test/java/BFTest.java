import com.peninsula.frc2023.config.ArmConstants;
import com.peninsula.frc2023.subsystems.Arm;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFKinematics;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class BFTest {
  @Test
  public void test() {

    TwoDOFKinematics.ArmConfiguration config =
        new TwoDOFKinematics.ArmConfiguration(
            Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-45));

    double motor1 = Arm.getInstance().joint1AngleToMotor(config.getAngle1());
    double motor2 = Arm.getInstance().joint2AngleToMotor(config.getAngle1(), config.getAngle2());

    Rotation2d a1 = Arm.getInstance().motorToJoint1Angle(motor1);
    Rotation2d a2 = Arm.getInstance().motorToJoint2FrameAngle(motor1, motor2);

    System.out.println("a1: " + a1);
    System.out.println("a2: " + a2);
  }

  @Test
  public void ff() {
    TwoDOFKinematics.ArmConfiguration d =
        new TwoDOFKinematics.ArmConfiguration(new Rotation2d(), new Rotation2d());
    double a = ArmConstants.torqueOnJ1(d) * ArmConstants.antiGravFeedforwardJ1Coeff;
    double b = ArmConstants.torqueOnJ2(d) * ArmConstants.antiGravFeedforwardJ2Coeff;
    System.out.println(a);
    System.out.println(b);
  }
}
