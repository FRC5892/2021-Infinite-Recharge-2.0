/*
 * Copied from LigerBots 2877 2021 Infinite Recharge Code, variable names changed
 * https://github.com/ligerbots/InfiniteRecharge2021/blob/main/src/main/java/frc/robot/simulation/SparkMaxWrapper.java
 * Create a wrapper around the CANSparkMax class to support simulation
 */

package frc.robot.simulationWrappers;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotController;

public class SparkMaxWrapper extends CANSparkMax {
    private SimDouble simSpeed;
    private SimDevice simSparkMax;

    public SparkMaxWrapper(int deviceID, MotorType type) {
        super(deviceID,type);

        simSparkMax = SimDevice.create("SparkMaxSim",deviceID);
        if (simSparkMax != null){
            simSpeed = simSparkMax.createDouble("speed", false, 0.0);
        }
    }

    @Override
    public double get(){
        if (simSparkMax != null){
            return simSpeed.get();
        }
        return super.get();
    }

    @Override
    public void set(double speed){
        if (simSparkMax != null){
            simSpeed.set(speed);
        }else{
            super.set(speed);
        }
    }

    @Override
    public void setVoltage(double outputVolts) { //For simulation purposes, we are expecting that the battery voltage stays constant.
        if (simSparkMax != null){
            set(outputVolts / RobotController.getBatteryVoltage());
        } else {
            super.setVoltage(outputVolts);
        }
    }
}