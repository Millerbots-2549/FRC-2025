// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import java.util.function.Consumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private static final int LED_COUNT = 29;
  private static final int[][] STRIP_BOUNDS = {
    {0, 28}
  };

  private final AddressableLED leds = new AddressableLED(0);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_COUNT);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {

  }

  private int getIndex(int stripIndex, int ledIndex) {
    return STRIP_BOUNDS[stripIndex][0] + ledIndex;
  }

  private boolean isLEDOutOfBounds(int stripIndex, int ledIndex) {
    return STRIP_BOUNDS[stripIndex][0] + ledIndex >= STRIP_BOUNDS[stripIndex][1];
  }

  public void setRGB(int stripIndex, int ledIndex, int r, int g, int b) {
    if (!isLEDOutOfBounds(stripIndex, ledIndex)) {
      buffer.setRGB(getIndex(stripIndex, ledIndex), r, g, b);
    }
  }

  public void setHSV(int stripIndex, int ledIndex, int h, int s, int v) {
    if (!isLEDOutOfBounds(stripIndex, ledIndex)) {
      buffer.setHSV(getIndex(stripIndex, ledIndex), h, s, v);
    }
  }

  public void setSegment(int segmentIndex, Color color) {
    for(int i = STRIP_BOUNDS[segmentIndex][0]; i < STRIP_BOUNDS[segmentIndex][1]; i++) {
      buffer.setLED(i, color);
    }
  }

  public void setSegmentGradient(int segmentIndex, Color color1, Color color2) {
    for(int i = STRIP_BOUNDS[segmentIndex][0]; i < STRIP_BOUNDS[segmentIndex][1]; i++) {
      double scale = (i - STRIP_BOUNDS[segmentIndex][0])
        / (STRIP_BOUNDS[segmentIndex][1] - STRIP_BOUNDS[segmentIndex][0]);
      buffer.setLED(i, Color.lerpRGB(color1, color2, scale));
    }
  }

  public void setSegmentMultiGradient(int segmentIndex, Pair<Color, Integer>[] colors) {
    int startIndex = STRIP_BOUNDS[segmentIndex][0];

    for(int num = 0; num < colors.length - 1; num++) {
      int partitionLength = (colors[num + 1].getSecond() - colors[num].getSecond());
      for(int i = 0; i < partitionLength; i++) {
        double scale = i / partitionLength;
        buffer.setLED(startIndex + colors[num].getSecond() + i,
          Color.lerpRGB(colors[num].getFirst(), colors[num + 1].getFirst(), scale));
      }
    }
  }

  public void setSegmentRainbow(int segmentIndex) {
    int stripLength = STRIP_BOUNDS[segmentIndex][1] - STRIP_BOUNDS[segmentIndex][0];

    for(int i = 0; i < stripLength; i++) {
      int hue = (0 + (i * 180 / stripLength)) % 180;
      this.setHSV(segmentIndex, i, hue, 255, 50);
    }
  }

  public void setSegmentAlternating(int segmentIndex, Color color1, Color color2) {
    setSegmentAlternating(segmentIndex, color1, color2, 2);
  }

  public void setSegmentAlternating(int segmentIndex, Color color1, Color color2, int frequency) {
    int stripLength = STRIP_BOUNDS[segmentIndex][1] - STRIP_BOUNDS[segmentIndex][0];

    for(int i = 0; i < Math.round(stripLength / frequency); i++) {
      for(int j = 0; j < frequency; j++) {
        buffer.setLED(STRIP_BOUNDS[segmentIndex][0] + (i * frequency) + j,
          MathUtil.inputModulus(j, 0, frequency) == 0 ? color1 : color2);
      }
    }
  }

  public void refresh() {
    leds.setData(buffer);
  }

  public void foreach(Consumer<Integer> consumer) {
    for(int i = 0; i < STRIP_BOUNDS.length; i++) {
      consumer.accept(i);
    }
  }

  @Override
  public void periodic() {
    refresh();
  }
}
