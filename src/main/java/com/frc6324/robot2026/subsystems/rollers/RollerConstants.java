package com.frc6324.robot2026.subsystems.rollers;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;

@UninstantiableClass
public final class RollerConstants {
  public static final CANBus ROLLER_CAN_BUS = Constants.RIO_BUS;
  public static final int ROLLER_LEADER_ID = 21;
  public static final int ROLLER_FOLLOWER_ID = 22;

  public static final TalonFXConfiguration ROLLER_MOTOR_CONFIG = new TalonFXConfiguration();
  public static final MotorAlignmentValue ROLLER_ALIGNMENT = MotorAlignmentValue.Opposed;
  public static final Current ROLLER_SPIN_CURRENT = Amps.of(100);

  public static final double ROLLER_MOI = 0.1;
  public static final double ROLLER_REDUCTION = 2;
  public static final MotorType ROLLER_MOTOR_TYPE = MotorType.KrakenX44;
  public static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX44Foc(1);

  private RollerConstants() {
    throw new IllegalAccessError();
  }
}
