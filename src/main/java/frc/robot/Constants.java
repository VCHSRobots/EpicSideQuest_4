// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.CANDeviceId;
import frc.lib.ServoMotorSubsystemConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class TurntableConstants {
    public static final double kPose1 = 10;

    public static final ServoMotorSubsystemConfig kTurntableConstants =
        new ServoMotorSubsystemConfig();

    static {
      kTurntableConstants.name = "Turntable";
      kTurntableConstants.fxConfig.Slot0.kP = 1;
      // kTurntableConstants.fxConfig.Slot1.kI = 0;
      // kTurntableConstants.fxConfig.Slot2.kD = 0;

      kTurntableConstants.talonCANID = new CANDeviceId(27);
      kTurntableConstants.unitToRotorRatio = ((1.0 / 9.4)  * 360.0);
      kTurntableConstants.fxConfig.Slot0.kP = .5;
      kTurntableConstants.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 2;
      kTurntableConstants.fxConfig.MotionMagic.MotionMagicAcceleration = 1;
      kTurntableConstants.fxConfig.MotionMagic.MotionMagicJerk = 0;
      kTurntableConstants.fxConfig.MotionMagic.MotionMagicExpo_kV = .15;
      kTurntableConstants.fxConfig.MotionMagic.MotionMagicExpo_kA = 5;
    }
  }

  public static class ActuatorConstants {
    static double m_bottomHole = .0058;
    static double m_middleHole = .02;
    static double m_topHole = .0385;
  }
}
