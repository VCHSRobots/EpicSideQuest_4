package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

/** Contains basic functions that are used often. */
public class Util {
  public static final double kEpsilon = 1e-12;

  /** Prevent this class from being instantiated. */
  private Util() {}

  /** Limits the given input to the given magnitude. */
  public static double limit(double v, double maxMagnitude) {
    return limit(v, -maxMagnitude, maxMagnitude);
  }

  public static double limit(double v, double min, double max) {
    return Math.min(max, Math.max(min, v));
  }

  public static int limit(int v, int min, int max) {
    return Math.min(max, Math.max(min, v));
  }

  public static boolean inRange(double v, double maxMagnitude) {
    return inRange(v, -maxMagnitude, maxMagnitude);
  }

  /** Checks if the given input is within the range (min, max), both exclusive. */
  public static boolean inRange(double v, double min, double max) {
    return v > min && v < max;
  }

  public static double interpolate(double a, double b, double x) {
    x = limit(x, 0.0, 1.0);
    return a + (b - a) * x;
  }

  public static String joinStrings(final String delim, final List<?> strings) {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < strings.size(); ++i) {
      sb.append(strings.get(i).toString());
      if (i < strings.size() - 1) {
        sb.append(delim);
      }
    }
    return sb.toString();
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(int a, int b, int epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
    boolean result = true;
    for (Double value_in : list) {
      result &= epsilonEquals(value_in, value, epsilon);
    }
    return result;
  }

  public static int orientation(double x1, double y1, double x2, double y2, double x3, double y3) {
    double val = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);
    if (val == 0) {
      return 0; // Colinear
    } else if (val > 0) {
      return 1; // Counter Clockwise
    } else {
      return 2; // Clockwise
    }
  }

  public static double handleDeadband(double value, double deadband) {
    deadband = Math.abs(deadband);
    if (deadband == 1) {
      return 0;
    }
    double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);
    return (Math.abs(value) > Math.abs(deadband)) ? scaledValue : 0;
  }

  public static Translation2d capVelocity(Translation2d velocity, double maxVelocity) {
    Translation2d cappedVelocity = velocity;
    if (velocity.getNorm() > maxVelocity) {
      cappedVelocity =
          new Translation2d(
              Math.cos(velocity.getAngle().getRadians()) * maxVelocity,
              Math.sin(velocity.getAngle().getRadians()) * maxVelocity);
    }
    return cappedVelocity;
  }

  public static void putPoseToSmartDashboard(String name, Pose2d pose) {
    SmartDashboard.putNumberArray(
        name, new Double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});
  }

  public static void putTranslationToSmartDashboard(String name, Translation2d translation) {
    SmartDashboard.putNumberArray(name, new Double[] {translation.getX(), translation.getY(), 0.0});
  }

  public static <T> Supplier<T> memoizeByIteration(IntSupplier iteration, Supplier<T> delegate) {
    AtomicReference<T> value = new AtomicReference<>();
    AtomicInteger last_iteration = new AtomicInteger(-1);
    return () -> {
      int last = last_iteration.get();
      int now = iteration.getAsInt();
      if (last != now) {
        value.updateAndGet((cur) -> null);
      }
      T val = value.get();
      if (val == null) {
        val = value.updateAndGet(cur -> cur == null ? delegate.get() : cur);
        last_iteration.set(now);
      }
      return val;
    };
  }

  public static boolean isDriverTryingToMove(CommandXboxController controller, double deadband) {
    return Double.compare(MathUtil.applyDeadband(controller.getLeftY(), deadband), 0.0) != 0
        || Double.compare(MathUtil.applyDeadband(controller.getLeftX(), deadband), 0.0) != 0
        || Double.compare(MathUtil.applyDeadband(controller.getRightX(), deadband), 0.0) != 0;
  }

  public static double cubic(double x, double weight) {
    return weight * x * x * x + (1.0 - weight) * x;
  }

  public static Translation2d calculateTranslationalVelocityWithDeadband(
      double leftX,
      double leftY,
      double maxVelocity,
      double exp,
      double db,
      double angularLockingDeadzoneDegrees) {

    if (leftX == 0 && leftY == 0) {
      return Translation2d.kZero;
    }

    /* find scaled velocity */
    Translation2d velocityTranslation = new Translation2d(leftX, leftY);
    double scaledVelocity =
        Math.pow(MathUtil.applyDeadband(velocityTranslation.getNorm(), db), exp) * maxVelocity;

    /* find velociy angle */
    double velocityRadians;
    double angleRadians = velocityTranslation.getAngle().getRadians();

    // round to nearest axis [-180, -90, 0, 90]
    double roundTo = Math.PI / 2.0;
    // divide measured angle by roundTo interval to get closest integer multiple of roundTo
    // multiply by roundTo to get the actual angle of the axis
    double closestAxis = roundTo * Math.round(angleRadians / roundTo);
    // convert angle Deadband to radians
    double angleDb = Units.degreesToRadians(angularLockingDeadzoneDegrees / 2.0);

    // scale angle, find difference to closest axis
    double angleDistanceToAxis = closestAxis - angleRadians;
    // apply deadband to the angle, using a max input value of roundTo/2
    // since the deadband should scale from each axis to the mid-angle between them equally
    // deadband returns a value [-1.0, 1.0]
    // multiply by half the distance in between axis,
    // since that's where the scaled values from each axis must be equal
    double scaledDifferenceRadians =
        (roundTo / 2.0) * MathUtil.applyDeadband(angleDistanceToAxis, angleDb, roundTo / 2.0);
    // re-reference the angle to the axis it was closest to
    double scaledAngleRadians = closestAxis - scaledDifferenceRadians;
    velocityRadians = scaledAngleRadians;

    return new Translation2d(scaledVelocity, Rotation2d.fromRadians(velocityRadians));
  }
}
