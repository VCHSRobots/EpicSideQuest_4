// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.CANDeviceId;
import frc.lib.MotorInputs;
import frc.lib.ServoMotorSubsystem;
import frc.lib.ServoMotorSubsystemConfig;
import frc.lib.TalonFXIO;

public class Turntable extends ServoMotorSubsystem<MotorInputs, TalonFXIO> {
  public Turntable(ServoMotorSubsystemConfig c, final TalonFXIO io) {
    super(c, new MotorInputs(), io);
    setCurrentPosition(0);
    this.setCurrentPositionAsZero();
  } 

  public Command goToSetpointCommand(double setpoint) {
    return positionSetpointUntilOnTargetCommand(() -> setpoint, () -> 3.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    SmartDashboard.putString(getSubsystem() + "/Command", getCurrentCommand() != null ? getCurrentCommand().getName() : "none");
  }
  }
