import edu.wpi.first.math.controller.PIDController;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class PIDTest {

  double epsilon = 1E-3;

  @Test
  public void testFull() {
    PIDController t = new PIDController(0.1, 0, 0.01);
    t.enableContinuousInput(-Math.PI, Math.PI);

    //    double z = t.calculate(0, 0);
    double z = t.calculate(Math.PI, Math.PI);

    System.out.println("z: " + z);
  }
}
