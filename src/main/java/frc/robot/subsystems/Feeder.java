// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.CANDeviceId;
import frc.lib.ServoMotorSubsystemConfig;
import frc.lib.TalonFXIO;

public class Feeder extends SubsystemBase {
  private TalonFXIO m_beltIO;

  private ServoMotorSubsystemConfig m_config = new ServoMotorSubsystemConfig();
  /** Creates a new Feeder. */
  public Feeder() {
    m_config.talonCANID = new CANDeviceId(3);
    m_config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    m_config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_beltIO = new TalonFXIO(m_config);
  }
  public void set(double value) {
    m_beltIO.setOpenLoopDutyCycle(value);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
