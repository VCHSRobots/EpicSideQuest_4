// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Actuator extends SubsystemBase {

  private TalonSRX m_actuator = new TalonSRX(22);
  CANcoder m_encoder = new CANcoder(7);
  private double setPoint;
  /** Creates a new Actuator. */
  public Actuator() {
    m_encoder.setPosition(0);
  }

  public Command goToSetpointCommand(double setPoint) { 
    return Commands.either(
      Commands.runEnd(() -> this.set(-.7), () -> this.set(0), this).raceWith(Commands.waitUntil(isCloseToSetpoint(setPoint))),
      Commands.runEnd(() -> this.set(.7), () -> this.set(0), this).raceWith(Commands.waitUntil(isCloseToSetpoint(setPoint))),
      () -> setPoint - m_encoder.getPosition().getValueAsDouble() > 0 ).alongWith(Commands.runOnce(() -> this.setPoint = setPoint));
  }

  private BooleanSupplier isCloseToSetpoint(double setPoint) {
    return () -> Math.abs(setPoint - m_encoder.getPosition().getValueAsDouble()) < .0005; //threshold
  }

  public void set(double value) {
    m_actuator.set(TalonSRXControlMode.PercentOutput, value);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("angle", m_encoder.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("isAtBot",  isCloseToSetpoint(setPoint).getAsBoolean());
    SmartDashboard.putNumber("Setpoint", setPoint);
    SmartDashboard.putString("ActuatorCmd", this.getCurrentCommand() != null ? this.getCurrentCommand().toString() : "none");
    // This method will be called once per scheduler run
  }
}
