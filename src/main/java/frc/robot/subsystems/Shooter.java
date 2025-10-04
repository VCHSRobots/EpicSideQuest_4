// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ServoMotorSubsystemConfig;
import frc.lib.TalonFXIO;

public class Shooter extends SubsystemBase {
  private TalonFXIO shooterTop;
  private TalonFXIO shooterBottom;
  private ServoMotorSubsystemConfig m_configtop = new ServoMotorSubsystemConfig();
  private ServoMotorSubsystemConfig m_configbot = new ServoMotorSubsystemConfig();
  /** Creates a new Shooter. */
  public Shooter() {
    m_configtop.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_configbot.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_configtop.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    m_configbot.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    shooterTop = new TalonFXIO(m_configtop);
    shooterBottom = new TalonFXIO(m_configbot);
  }

  public void set(double value) {
    shooterTop.setOpenLoopDutyCycle(value);
    shooterBottom.setOpenLoopDutyCycle(value);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
