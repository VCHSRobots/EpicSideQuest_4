// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.CANDeviceId;
import frc.lib.ServoMotorSubsystemConfig;
import frc.lib.TalonFXIO;

public class Shooter extends SubsystemBase {
  
  private TalonFXIO shooterTop;
  private TalonFXIO shooterBottom;

  ServoMotorSubsystemConfig m_topConfig = new ServoMotorSubsystemConfig();
  ServoMotorSubsystemConfig m_bottomConfig = new ServoMotorSubsystemConfig();
  /** Creates a new Shooter. */
  public Shooter() {
    m_topConfig.talonCANID = new CANDeviceId(0);
    m_bottomConfig.talonCANID = new CANDeviceId(1);
    m_topConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    m_bottomConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    m_topConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterTop = new TalonFXIO(m_topConfig);
    shooterBottom = new TalonFXIO(m_bottomConfig);
  }

  
  public void set(double value) {
    shooterTop.setOpenLoopDutyCycle(value);
    shooterBottom.setOpenLoopDutyCycle(value-.1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
