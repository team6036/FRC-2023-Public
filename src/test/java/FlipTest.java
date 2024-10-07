import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import org.junit.Test;

public class FlipTest {
  @Test
  public void test() {
    PathPlannerTrajectory traj1 = PathPlanner.loadPath("Test1", 2, 2);
    Pose2d inital = traj1.getInitialHolonomicPose();

    System.out.println(inital);
  }
}
