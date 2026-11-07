package com.frc6324.robot2026.subsystems;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class JetsonDiagnostics extends SubsystemBase {
  private static final String LOG_PREFIX = "Jetson/";

  private final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("photonvision/jetson");
  private final Map<String, GenericSubscriber> subscribers = new HashMap<>();

  @Override
  public void periodic() {
    final List<Topic> topics = table.getTopics();

    Logger.recordOutput(LOG_PREFIX + "TopicCount", topics.size());

    for (final Topic topic : topics) {
      final String name = topic.getName().substring(table.getPath().length() + 1);
      final GenericSubscriber subscriber =
          subscribers.computeIfAbsent(name, n -> topic.genericSubscribe());
      final NetworkTableValue value = subscriber.get();
      final String key = LOG_PREFIX + name;

      switch (value.getType()) {
        case kBoolean -> Logger.recordOutput(key, value.getBoolean());
        case kBooleanArray -> Logger.recordOutput(key, value.getBooleanArray());
        case kDouble -> Logger.recordOutput(key, value.getDouble());
        case kDoubleArray -> Logger.recordOutput(key, value.getDoubleArray());
        case kFloat -> Logger.recordOutput(key, value.getFloat());
        case kFloatArray -> Logger.recordOutput(key, value.getFloatArray());
        case kInteger -> Logger.recordOutput(key, value.getInteger());
        case kIntegerArray -> Logger.recordOutput(key, value.getIntegerArray());
        case kString -> Logger.recordOutput(key, value.getString());
        case kStringArray -> Logger.recordOutput(key, value.getStringArray());
        // Raw or unassigned doesn't give us enough info about the data to log, so skip
        default -> {}
      }
    }
  }
}
