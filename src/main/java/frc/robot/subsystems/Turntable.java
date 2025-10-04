// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.MotorInputs;
import frc.lib.ServoMotorSubsystem;
import frc.lib.ServoMotorSubsystemConfig;
import frc.lib.TalonFXIO;

public class Turntable extends ServoMotorSubsystem<MotorInputs, TalonFXIO> {
    // m_turntable = new TalonFXIO(c);
  public Turntable(ServoMotorSubsystemConfig c, final TalonFXIO io) {
    super(c, new MotorInputs(), io);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }
  }
