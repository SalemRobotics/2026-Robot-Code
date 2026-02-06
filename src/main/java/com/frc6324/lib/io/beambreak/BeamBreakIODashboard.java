package com.frc6324.lib.io.beambreak;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public final class BeamBreakIODashboard implements BeamBreakIO {
    private final LoggedNetworkBoolean networkBoolean;

    public BeamBreakIODashboard(String key) {
        networkBoolean = new LoggedNetworkBoolean(key);
    }

    @Override
    public void updateInputs(BeamBreakInputs inputs) {
        inputs.connected = true;
        inputs.broken = networkBoolean.get();
    }
}
