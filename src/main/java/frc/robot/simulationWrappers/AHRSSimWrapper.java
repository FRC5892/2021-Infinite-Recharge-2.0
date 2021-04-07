/*
 * Copied from LigerBots 2877 2021 Infinite Recharge Code, variable names changed
 * https://github.com/ligerbots/InfiniteRecharge2021/blob/main/src/main/java/frc/robot/simulation/AHRSSimWrapper.java
 * Create a wrapper around the AHRS (navX) class to support simulation
 */

package frc.robot.simulationWrappers;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

public class AHRSSimWrapper extends AHRS {
  // Need to have a simulated device, plus some places to hold the values
  private SimDevice simDevice;
  private SimDouble simAngle = null;
  private SimDouble simRate = null;
  private SimDouble simPitch = null;
 
  public AHRSSimWrapper(SPI.Port kmxp, byte updateRate_hz) {
    super(kmxp, updateRate_hz);

    // Create the SimDevice. If it returns null, we are not in simulation
    simDevice = SimDevice.create("AHRS", kmxp.value);
    if (simDevice != null) {
      simAngle = simDevice.createDouble("Angle", false, 0.0);
      simRate = simDevice.createDouble("Rate", false, 0.0);
      simPitch = simDevice.createDouble("Pitch", false, 0.0);
    }
  }

  @Override
  public double getAngle() {
    if (simAngle != null) {
      return simAngle.get();
    }
    return super.getAngle();
  }

  @Override
  public double getRate() {
    if (simRate != null) {
      return simRate.get();
    }
    return super.getRate();
  }

  @Override
  public float getPitch() {
    if (simPitch != null) {
      return (float)simPitch.get();
    }
    return super.getPitch();
  }
}