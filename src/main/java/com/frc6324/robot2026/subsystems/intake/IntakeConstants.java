package com.frc6324.robot2026.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;

@UninstantiableClass
public final class IntakeConstants {
  private IntakeConstants() throws Error {
    throw new IllegalAccessError();
  }

  public static final CANBus INTAKE_CAN_BUS = Constants.CANIVORE;
  public static final int EXTENSION_MOTOR_ID = 20;
  public static final int SPIN_LEADER_MOTOR_ID = 21;
  public static final int SPIN_FOLLOWER_MOTOR_ID = 22;

  public static final Distance INTAKE_WIDTH = Inches.of(27);

  public static final TalonFXConfiguration EXTENSION_MOTOR_CONFIG = new TalonFXConfiguration();
  public static final TalonFXConfiguration SPIN_LEADER_MOTOR_CONFIG = new TalonFXConfiguration();
  public static final TalonFXConfiguration SPIN_FOLLOWER_MOTOR_CONFIG = new TalonFXConfiguration();

  public static final double INTAKE_MOI = 0;
  public static final double INTAKE_REDUCTION = 9;
  public static final DCMotor INTAKE_GEARBOX = DCMotor.getKrakenX60Foc(1);
}
