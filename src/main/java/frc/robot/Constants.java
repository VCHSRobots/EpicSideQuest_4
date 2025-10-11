// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

      kTurntableConstants.talonCANID = new CANDeviceId(9);
      kTurntableConstants.unitToRotorRatio = (8 ); // (1 / 7.721) * 360;
      kTurntableConstants.fxConfig.Slot0.kP = 1;
      // kTurntableConstants.fxConfig.MotionMagi
    }
  }
}
