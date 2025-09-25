package frc.lib;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.lib.Util;
import java.util.function.DoubleSupplier;

/**
 * RollerMotorSubsystem
 *
 * @param <T>
 */
public class ServoMotorSubsystem<T extends MotorInputs, U extends TalonFXIO> extends SubsystemBase {
  public U io;
  public T inputs;
  private double positionSetpoint = 0.0;

  protected ServoMotorSubsystemConfig conf;

  private NetworkTableEntry latencyPeriodicSecLogEntry;
  private NetworkTableEntry openLoopDutyCycleLogEntry;
  private NetworkTableEntry positionSetpointLogEntry;
  private NetworkTableEntry neutralModeLogEntry;
  private NetworkTableEntry motionMagicSetpointLogEntry;
  private NetworkTableEntry velocitySetpointLogEntry;
  private NetworkTableEntry positionLogEntry;
  private NetworkTableEntry isAtSetpointEntry;

  public ServoMotorSubsystem(ServoMotorSubsystemConfig config, T inputs, U io) {
    super(config.name);
    this.conf = config;
    this.io = io;
    this.inputs = inputs;

    latencyPeriodicSecLogEntry = SmartDashboard.getEntry(getName() + "/latencyPeriodicSec");
    openLoopDutyCycleLogEntry =
        SmartDashboard.getEntry(getName() + "/API/setOpenLoopDutyCycle/dutyCycle");
    positionSetpointLogEntry =
        SmartDashboard.getEntry(getName() + "/API/setPositionSetpointImp/Units");
    neutralModeLogEntry = SmartDashboard.getEntry(getName() + "/API/setNeutralModeImpl/Mode");
    motionMagicSetpointLogEntry =
        SmartDashboard.getEntry(getName() + "/API/setMotionMagicSetpointImp/Units");
    velocitySetpointLogEntry =
        SmartDashboard.getEntry(getName() + "/API/setVelocitySetpointImpl/UnitsPerS");
    latencyPeriodicSecLogEntry.setDouble(0);
    positionLogEntry = SmartDashboard.getEntry(getName() + "/positionUnits");
    isAtSetpointEntry = SmartDashboard.getEntry(getName() + "/isAtSetpoint");

    latencyPeriodicSecLogEntry.setDouble(0);
    openLoopDutyCycleLogEntry.setDouble(0);
    positionSetpointLogEntry.setDouble(0);
    neutralModeLogEntry.setDouble(0);
    motionMagicSetpointLogEntry.setDouble(0);
    velocitySetpointLogEntry.setDouble(0);
    latencyPeriodicSecLogEntry.setDouble(0);
    positionLogEntry.setDouble(0);

    setDefaultCommand(dutyCycleCommand(() -> 0.0).withName(getName() + " Default Command Neutral"));
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    io.readInputs(inputs);

    isAtSetpointEntry.setBoolean(isAtSetpoint());

    positionLogEntry.setNumber(inputs.unitPosition);

    latencyPeriodicSecLogEntry.setNumber(RobotTime.getTimestampSeconds() - timestamp);

    // Logger.processInputs(getName(), inputs);
    SmartDashboard.putNumber(getName() + "/inputs/appliedVolts", inputs.appliedVolts);
    SmartDashboard.putNumber(getName() + "/inputs/currentStatorAmps", inputs.currentStatorAmps);
    SmartDashboard.putNumber(getName() + "/inputs/currentSupplyAmps", inputs.currentSupplyAmps);
    SmartDashboard.putNumber(getName() + "/inputs/unitPosition", inputs.unitPosition);
    SmartDashboard.putNumber(getName() + "/inputs/velocityUnitsPerSecond", inputs.velocityUnitsPerSecond);
  }

  protected void setOpenLoopDutyCycleImpl(double dutyCycle) {
    openLoopDutyCycleLogEntry.setNumber(dutyCycle);
    io.setOpenLoopDutyCycle(dutyCycle);
  }

  protected void setPositionSetpointImpl(double units) {
    positionSetpoint = units;
    positionSetpointLogEntry.setNumber(units);
    io.setPositionSetpoint(units);
  }

  protected void setNeutralModeImpl(NeutralModeValue mode) {
    neutralModeLogEntry.setString(mode.name());
    io.setNeutralMode(mode);
  }

  protected void setMotionMagicSetpointImpl(double units) {
    positionSetpoint = units;
    motionMagicSetpointLogEntry.setNumber(units);
    io.setMotionMagicSetpoint(units);
  }

  protected void setVelocitySetpointImpl(double unitsPerSecond) {
    velocitySetpointLogEntry.setNumber(unitsPerSecond);
    io.setVelocitySetpoint(unitsPerSecond);
  }

  public double getCurrentPosition() {
    return inputs.unitPosition;
  }

  public double getCurrentVelocity() {
    return inputs.velocityUnitsPerSecond;
  }

  public double getPositionSetpoint() {
    return positionSetpoint;
  }

  protected boolean isAtSetpoint() {
    // if (Robot.isSimulation()) {
    //   return true;
    // }
    return Util.epsilonEquals(getCurrentPosition(), positionSetpoint, this.conf.toleranceUnits);
  }

  public Command dutyCycleCommand(DoubleSupplier dutyCycle) {
    return runEnd(
            () -> {
              setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
            },
            () -> {
              setOpenLoopDutyCycleImpl(0.0);
            })
        .withName(getName() + " DutyCycleControl");
  }

  public Command velocitySetpointCommand(DoubleSupplier velocitySupplier) {
    return runEnd(
            () -> {
              setVelocitySetpointImpl(velocitySupplier.getAsDouble());
            },
            () -> {})
        .withName(getName() + " VelocityControl");
  }

  public Command setCoast() {
    return startEnd(
            () -> setNeutralModeImpl(NeutralModeValue.Coast),
            () -> setNeutralModeImpl(NeutralModeValue.Brake))
        .withName(getName() + "CoastMode")
        .ignoringDisable(true);
  }

  public Command positionSetpointCommand(DoubleSupplier unitSupplier) {
    return runEnd(
            () -> {
              setPositionSetpointImpl(unitSupplier.getAsDouble());
            },
            () -> {})
        .withName(getName() + " positionSetpointCommand");
  }

  public Command positionSetpointUntilOnTargetCommand(
      DoubleSupplier unitSupplier, DoubleSupplier epsilon) {
    return new ParallelDeadlineGroup(
        new WaitUntilCommand(
            () ->
                Util.epsilonEquals(
                    unitSupplier.getAsDouble(), inputs.unitPosition, epsilon.getAsDouble())),
        positionSetpointCommand(unitSupplier));
  }

  public Command motionMagicSetpointCommand(DoubleSupplier unitSupplier) {
    return runEnd(
            () -> {
              setMotionMagicSetpointImpl(unitSupplier.getAsDouble());
            },
            () -> {})
        .withName(getName() + " motionMagicSetpointCommand");
  }

  public Command motionMagicUntilSetpointCommand(DoubleSupplier unitSupplier) {
    return runOnce(() -> setMotionMagicSetpointImpl(unitSupplier.getAsDouble()))
        .andThen(
            motionMagicSetpointCommand(() -> unitSupplier.getAsDouble()).until(this::isAtSetpoint));
  }

  protected void setCurrentPositionAsZero() {
    io.setCurrentPositionAsZero();
  }

  protected void setCurrentPosition(double positionUnits) {
    io.setCurrentPosition(positionUnits);
  }

  public Command waitUntilTargetPositionCommand(DoubleSupplier targetPosition) {
    return new WaitUntilCommand(
        () ->
            Util.epsilonEquals(
                inputs.unitPosition, targetPosition.getAsDouble(), this.conf.toleranceUnits));
  }

  public Command waitUntilAboveTargetPosition(double targetPosition) {
    return new WaitUntilCommand(
        () -> {
          // if (Robot.isSimulation()) {
          //   return true;
          // }
          return inputs.unitPosition > targetPosition;
        });
  }

  public Command waitUntilBelowTargetPosition(double targetPosition) {
    return new WaitUntilCommand(() -> inputs.unitPosition < targetPosition);
  }

  public Command waitUntilAboveTargetPosition(DoubleSupplier targetPosition) {
    return new WaitUntilCommand(() -> inputs.unitPosition > targetPosition.getAsDouble());
  }

  public Command waitUntilBelowTargetPosition(DoubleSupplier targetPosition) {
    return new WaitUntilCommand(() -> inputs.unitPosition < targetPosition.getAsDouble());
  }

  public Command waitUntilSetpointCommand() {
    return new WaitUntilCommand(this::isAtSetpoint);
  }

  protected Command withoutLimitsTemporarily() {
    var prev =
        new Object() {
          boolean fwd = false;
          boolean rev = false;
        };
    return Commands.startEnd(
        () -> {
          prev.fwd = conf.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable;
          prev.rev = conf.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable;
          io.setEnableSoftLimits(false, false);
        },
        () -> {
          io.setEnableSoftLimits(prev.fwd, prev.rev);
        });
  }
}
