// package org.usfirst.frc.team4069.robot;

/**
 * Hybrid class this is an example of how to have the robot finish a set of moves, turn the turret to target and shoot when rpms are ok
 * 
 * @author EA
 *
 */
/*
 * public class Control_MoveAimShoot { private int mEnabled = 0; private Robot mRobot = null;
 * 
 * private static final int WAITING_MOVE_FINISH = 1; // NOTE these are bit states, more than one can be true at once private static final int WAITING_TURRET_AIMED = 2; private static final int WAITING_SHOOTER_RPM = 4; private int mState = 0;
 * 
 * public Control_MoveAimShoot(Robot robot) { mRobot = robot; }
 * 
 * public int Enable() { mEnabled = 1; return mEnabled; }
 * 
 * public int Disable() { mEnabled = 0; return mEnabled; }
 * 
 * public void Tick() { // first gather states mState = 0; if (mRobot.mMoveController.isControlMoveFinished() == 0) { mState |= WAITING_MOVE_FINISH; } if (mRobot.mTurretController.isTurretTargeted() == 0) { mState |= WAITING_TURRET_AIMED; } if
 * (mRobot.mShooterController.isShooterWithingPercentage(5) == 0) { mState |= WAITING_SHOOTER_RPM; }
 * 
 * if ((mState & WAITING_MOVE_FINISH) != 0) // not done move command list yet return;
 * 
 * mRobot.mTurretController.Enable(); // start targeting with turret
 * 
 * if ((mState & WAITING_TURRET_AIMED) != 0) // turret not locked on return;
 * 
 * mRobot.mShooterController.setRPMWanted(1800); // this should come from a table, we should get LIDAR distance here!
 * 
 * if ((mState & WAITING_SHOOTER_RPM) != 0) // shooter not within 5% of wanted rpm return;
 * 
 * // NOTE If there was a inteake to 'feed the shooter' we would turn it on here to feed balls into shooter... // does not exist yet }// tick } // Control_MoveAimShoot
 */