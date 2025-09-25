package frc.lib;

import edu.wpi.first.wpilibj.RobotController;

// import org.littletonrobotics.junction.Logger;

public class RobotTime {
  public static double getTimestampSeconds() {
    // long micros = Logger.getTimestamp();
    long micros = RobotController.getFPGATime();
    return (double) micros * 1.0E-6;
  }
}
