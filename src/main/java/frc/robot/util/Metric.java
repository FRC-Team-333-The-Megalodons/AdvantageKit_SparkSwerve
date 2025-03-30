package frc.robot.util;

import org.littletonrobotics.junction.Logger;

public class Metric {
  long total = 0, count = 0, lastStart = -1, lastLog = -1;
  boolean running = false;

  String logPath;

  public Metric(String component, String property) {
    logPath = "Vision/Summary/Metrics/" + component + "/" + property;
  }

  public void start() {
    if (running) {
      return;
    }
    running = true;
    lastStart = System.currentTimeMillis();
  }

  public void stop() {
    if (!running || lastStart < 0) {
      return;
    }
    running = false;
    long elapsed = System.currentTimeMillis() - lastStart;
    total += elapsed;
    count += 1;
  }

  public void log() {
    if (count < 1) {
      return;
    }
    lastLog = System.currentTimeMillis();
    double result = (double) total / (double) count;
    Logger.recordOutput(logPath, result);
  }

  final long LOG_THROTTLE_MS = 1000;

  public void logThrottled() {
    if (lastLog > 0 && System.currentTimeMillis() - lastLog < LOG_THROTTLE_MS) {
      // too soon to log again
      return;
    }

    log();
  }
}
