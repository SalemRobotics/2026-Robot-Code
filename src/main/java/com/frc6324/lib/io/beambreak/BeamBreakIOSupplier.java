package com.frc6324.lib.io.beambreak;

import java.util.function.BooleanSupplier;

public final class BeamBreakIOSupplier implements BeamBreakIO {
    private final BooleanSupplier supplier;

    public BeamBreakIOSupplier(BooleanSupplier supplier) {
        this.supplier = supplier;
    }

    @Override
    public void updateInputs(BeamBreakInputs inputs) {
        try {
            inputs.connected = true;
            inputs.broken = supplier.getAsBoolean();
        } catch (Exception e) {
            inputs.connected = false;
            throw e;
        }
    }
}
