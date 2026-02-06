package com.frc6324.lib.io.beambreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
    void updateInputs(BeamBreakInputs inputs);

    @AutoLog
    class BeamBreakInputs {
        public boolean connected = false;
        public boolean broken = false;
    }
}
