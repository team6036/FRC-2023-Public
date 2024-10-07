import com.peninsula.frc2023.config.ArmConstants;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFTorques;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class MqqCqqTgTest {
  @Test
  public void test() {
    TwoDOFTorques torques = ArmConstants.torquesFF;
    run(torques);
  }

  public void run(TwoDOFTorques torques) {
    TwoDOFTorques.Desired des = new TwoDOFTorques.Desired(0.0, 0.0, 1.0, 1);

    SimpleMatrix feedforwardAngle =
        torques.motorFeedforward(new TwoDOFTorques.State(0.0, 0.0, 0.0, 0.0), des);

    //    System.out.print(feedforwardAngle);
  }
}
