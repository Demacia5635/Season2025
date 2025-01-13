// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.utils;

/** Add your docs here. */
public class ArmState {
    private double baseJoint;
    private double griperJoint;

    public ArmState(double baseJoint, double griperJoint){
        this.baseJoint = baseJoint;
        this.griperJoint = griperJoint;
    }

    public void setState(double baseJoint, double griperJoint){
        this.baseJoint = baseJoint;
        this.griperJoint = griperJoint;
    }
    
    public ArmState getState(){
        return this;
    }

    public double baseJoint(){
        return this.baseJoint;
    }

    public double griperJoint(){
        return this.griperJoint;
    }

}
