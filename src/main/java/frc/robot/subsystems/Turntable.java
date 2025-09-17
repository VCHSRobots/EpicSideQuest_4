// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ServoMotorSubsystemConfig;
import frc.lib.TalonFXIO;

public class Turntable extends SubsystemBase {
  private TalonFXIO m_turntable;
  /** Creates a new Turntable. */
  public Turntable(ServoMotorSubsystemConfig c) {
    m_turntable = new TalonFXIO(c);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
