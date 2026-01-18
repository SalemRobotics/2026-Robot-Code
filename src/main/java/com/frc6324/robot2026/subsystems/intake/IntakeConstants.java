package com.frc6324.robot2026.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import com.frc6324.lib.UninstantiableClass;

import edu.wpi.first.units.measure.Distance;

@UninstantiableClass
public final class IntakeConstants {
    private IntakeConstants() throws Error {
        throw new IllegalAccessError();
    }

    public static final int MOTOR_ID = 20;

    public static final Distance INTAKE_WIDTH = Inches.of(27);
}
