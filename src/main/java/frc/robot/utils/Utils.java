package frc.robot.utils;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.springframework.context.ApplicationContext;
import org.springframework.context.support.ClassPathXmlApplicationContext;
import org.springframework.context.support.FileSystemXmlApplicationContext;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.robot.Robot;

public class Utils {

  private static ApplicationContext _context;

  /**
   * Gets either the real or mock class of the instance matching the beanName string - depending on build mode
   * @param beanName - the name of the instance one wants to get
   * @return the instance matching the beanName string - either mock or real - depending on build mode.
   */
  public static Object getBean(String beanName) {
    if (_context == null) {
      if (Robot.isReal()) {
        _context = new FileSystemXmlApplicationContext("/home/lvuser/deploy/DeployApllicationContext.xml");
      } else _context = new ClassPathXmlApplicationContext("/TestApplicationContext.xml");
    }
    return _context.getBean(beanName);
  }

  /**
   * Turn a given trajectory into JSON and write it to a file
   * @param trajectoryPath  The directory in which to write the JSONified trajectory
   * @param trajectory  The trajectory to JSONify
   */
  public static void serializeTrajectory(Path trajectoryPath, Trajectory trajectory) {
    try {
      new ObjectMapper().writeValue(new File(trajectoryPath.toString()), trajectory.getStates());
      System.out.println("Trajectory Succesfully written to file");
    } catch (IOException e) {
      System.out.println("Trajectory File not found / no write permission");
      e.printStackTrace();
    }
  }

  /**
   * Read JSON from a given file and turn it to a Trajectory object 
   * @param fileName the trajectory's directory
   * @return The deJSONified trajectory object
   */
  public static Trajectory deserializeTrajectory(Path trajectoryPath) {
    List<State> trajectoryStates = new ArrayList<State>();
    try {
      trajectoryStates = new ObjectMapper().readValue(
        new File(trajectoryPath.toString()),
        new TypeReference<List<State>>() {}
      );
    } catch (JsonProcessingException e) {
      System.out.println("Trajectory JSON invalid");
      e.printStackTrace();
    } catch (IOException e) {
      System.out.println("Trajectory File not found / no reading permission");
      e.printStackTrace();
    }
    return new Trajectory(trajectoryStates);
  }

  /**
   * 
   * @param x - value to check
   * @param L - the limit to check proximity to
   * @param tolerance - the tolerance q epsilon of this check
   * @return is x in L's neighborhood (of epsilon = tolerance)
   */
  public static boolean isInNeighborhood(double x, double L, double tolerance) {
    return Math.abs(x - L) < tolerance;
  }

  /**
   * 
   * @param units - the units from the sensor
   * @param upd - units per degree from the sensor
   * @return degrees - between 180 and -180
   */
  public static double unitsToDegs(double units, double upd) {
    return (((units / upd) % 360) + 360) % 360 - 180;
  }
}
