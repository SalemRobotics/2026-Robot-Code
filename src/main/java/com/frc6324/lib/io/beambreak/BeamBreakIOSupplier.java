package com.frc6324.lib.io.beambreak;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A beam break sensor that uses a Java function to determine the value reported by the "sensor" or simulation implementation.
 */
public final class BeamBreakIOSupplier implements BeamBreakIO {
    private final BooleanSupplier supplier;

    /**
     * Creates a new implementation of a "beam break" sensor given a function that returns a boolean.
     * 
     * <p>This implementation is meant to be used during non-replay simulation.
     * @param supplier The function to call periodically that returns if the sensor is broken.
     */
    public BeamBreakIOSupplier(BooleanSupplier supplier) {
        this.supplier = supplier;
    }

    /**
     * Creates a new implementation of a "beam break" sensor given a function that returns a boolean.
     * 
     * <p>This implementation is meant to be used during non-replay simulation.
     * @param supplier The function to call periodically that returns if the sensor is broken.
     */
    public BeamBreakIOSupplier(Supplier<Boolean> supplier) {
        // Simply wrap the supplier to get the underlying primitive
        this(() -> supplier.get().booleanValue());
    }

    @Override
    public void updateInputs(BeamBreakInputs inputs) {
        try {
            // Try to call the function
            inputs.connected = true;
            inputs.broken = supplier.getAsBoolean();
        } catch (Exception e) {
            // If the function throws an exception, propogate the exception and mark the sensor as disconnected.
            inputs.connected = false;
            throw e;
        }
    }
}
