package frc.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ServoMotorSubsystemConfig {

  public static class FollowerConstants {
    public CANDeviceId id = new CANDeviceId(-1);
    public boolean opposeMaster = false;
  }

  public String name = "UNNAMED";
  public CANDeviceId talonCANID;
  public TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  // Ratio of rotor to units for this talon. rotor * by this ratio should
  // be the units.
  // <1 is reduction
  public double unitToRotorRatio = 1.0;
  public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
  public double kMaxPositionUnits = Double.POSITIVE_INFINITY;

  // Moment of Inertia (KgMetersSquared) for sim
  public double momentOfInertia = 0.5;
  public double toleranceUnits = 1.0;
}
