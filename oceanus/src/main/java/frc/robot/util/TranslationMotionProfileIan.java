package frc.robot.util;

import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.vectors.filters.VFilter;
import com.stuypulse.stuylib.util.StopWatch;

public class TranslationMotionProfileIan implements VFilter {

    // Default number of times to apply filter (helps accuracy)
    private static final int kDefaultSteps = 64;

    // Stopwatch to Track dt
    private StopWatch mTimer;

    // Limits for each of the derivatives
    private Number mVelLimit;
    private Number mAccelLimit;

    // The last output / velocity
    private Vector2D mOutput;
    private Vector2D mVelocity;

    // Number of times to apply filter (helps accuracy)
    private final int mSteps;

    public TranslationMotionProfileIan(
        Number velLimit, 
        Number accelLimit, 
        Vector2D startingTranslation, 
        Vector2D startingVelocity, 
        int steps) 
    {
        mTimer = new StopWatch();

        mVelLimit = velLimit;
        mAccelLimit = accelLimit;

        mOutput = startingTranslation;
        mVelocity = startingVelocity;

        mSteps = steps;
    }

    public TranslationMotionProfileIan(Number velLimit, Number accelLimit, Vector2D startingTranslation, Vector2D startingVelocity) {
        this(velLimit, accelLimit, startingTranslation, startingVelocity, kDefaultSteps);
    }

    public TranslationMotionProfileIan(Number velLimit, Number accelLimit) {
        this(velLimit, accelLimit, Vector2D.kOrigin, Vector2D.kOrigin, kDefaultSteps);
    }

    public Vector2D get(Vector2D target) {
        double dt = mTimer.reset() / mSteps;

        for (int i = 0; i < mSteps; ++i) {
            // if there is a accel limit, limit the amount the velocity can change
            if (0 < mAccelLimit.doubleValue()) {
                // amount of windup in system (how long it would take to slow down)
                double windup = mVelocity.magnitude() / mAccelLimit.doubleValue();

                // If the windup is too small, just use normal algorithm to limit acceleration
                if (windup < dt) {
                    // Calculate acceleration needed to reach target
                    Vector2D accel = target.sub(mOutput).div(dt).sub(mVelocity);

                    // Try to reach it while abiding by accel limit
                    mVelocity = mVelocity.add(accel.clamp(dt * mAccelLimit.doubleValue()));
                } else {
                    // the position it would end up if it attempted to come to a full stop
                    Vector2D windA =
                        mVelocity.mul(0.5 * (dt + windup)); // windup caused by acceleration
                    Vector2D future = mOutput.add(windA); // where the robot will end up

                    // Calculate acceleration needed to come to stop at target throughout windup
                    Vector2D accel = target.sub(future).div(windup);

                    // Try to reach it while abiding by accel limit
                    mVelocity = mVelocity.add(accel.clamp(dt * mAccelLimit.doubleValue()));
                }

            } else {
                // make the velocity the difference between target and current
                mVelocity = target.sub(mOutput).div(dt);
            }

            // if there is an velocity limit, limit the velocity
            if (0 < mVelLimit.doubleValue()) {
                mVelocity = mVelocity.clamp(mVelLimit.doubleValue());
            }

            // adjust output by calculated velocity
            mOutput = mOutput.add(mVelocity.mul(dt));
        }

        // Field.FIELD2D.getObject("Translation Motion Profile Ian").setPose(!Robot.isBlue()
        //     ? new Pose2d(mOutput.x, mOutput.y, new Rotation2d())
        //     : Field.transformToOppositeAlliance(new Pose2d(mOutput.x, mOutput.y, new Rotation2d())));
            
        return mOutput;
    }
}
