package no.sonat.game

import no.sonat.game.geometry.LineSegment2D
import no.sonat.game.geometry.Vec2D
import org.slf4j.Logger
import org.slf4j.LoggerFactory

val logger: Logger = LoggerFactory.getLogger("Main")

fun main() {
    logger.info("Start client")
    val ag = AgentClient(
        wsUri = "ws://spacehex.norwayeast.cloudapp.azure.com:7070/test",
        //wsUri = "ws://spacehex.norwayeast.cloudapp.azure.com:7070/play",
        room = "j1ycg", //Not in use for test runs
        name = "Team kOtlin",
        strategy = ::calculateAcceleration,
        joinAction = {
            logger.info(it)
        }
    )
}

/**
 * Calculates the acceleration to navigate the lander to the goal while avoiding obstacles.
 * 
 * @param env The environment containing the goal and obstacles
 * @param lander The current state of the lander
 * @return The acceleration to apply to the lander
 */
fun calculateAcceleration(env: Environment, lander: Lander): Acceleration {
    // If the lander has crashed or completed, don't do anything
    if (lander.status != LanderStatus.FLYING) {
        return Acceleration(up = false, left = false, right = false)
    }

    // Calculate vector to goal
    val toGoal = env.goal - lander.position
    val distanceToGoal = toGoal.length()

    // Calculate safe landing parameters
    val maxSafeLandingVelocity = 10.0 // Maximum safe landing velocity
    val approachDistance = 50.0 // Distance to start slowing down for landing

    // Check if we're close to the goal and need to slow down for landing
    val needToSlowDown = distanceToGoal < approachDistance && 
                         lander.velocity.length() > maxSafeLandingVelocity

    // Check for obstacles in the path
    val obstacleAvoidanceVector = calculateObstacleAvoidance(env, lander)

    // Determine desired direction (combining goal direction and obstacle avoidance)
    val desiredDirection = if (obstacleAvoidanceVector != Vec2D.ZERO) {
        // If there's an obstacle, prioritize avoiding it
        (toGoal.unit() + obstacleAvoidanceVector * 2.0).unit()
    } else {
        // Otherwise head straight for the goal
        toGoal.unit()
    }

    // Determine acceleration based on desired direction and current velocity
    val up = (desiredDirection.y > 0 && lander.velocity.y < 20.0) || // Accelerate up if we need to go up
             (lander.velocity.y < -10.0) || // Counter gravity if falling too fast
             (needToSlowDown && lander.velocity.y < 0) // Slow down for landing

    val left = desiredDirection.x < 0 && lander.velocity.x > -15.0 // Go left if needed and not too fast
    val right = desiredDirection.x > 0 && lander.velocity.x < 15.0 // Go right if needed and not too fast

    return Acceleration(up = up, left = left, right = right)
}

/**
 * Calculates a vector to help avoid obstacles.
 * 
 * @param env The environment containing obstacles
 * @param lander The current state of the lander
 * @return A vector pointing away from nearby obstacles
 */
fun calculateObstacleAvoidance(env: Environment, lander: Lander): Vec2D {
    val position = lander.position
    val velocity = lander.velocity

    // Look ahead based on current velocity to predict future position
    val lookAheadDistance = velocity.length() * 1.5
    val predictedPath = LineSegment2D(
        position, 
        position + velocity.unit() * lookAheadDistance
    )

    // Check for potential collisions with ground segments
    var closestIntersection: Vec2D? = null
    var minDistance = Double.MAX_VALUE

    for (segment in env.segments) {
        val intersection = predictedPath.intersects(segment)
        if (intersection != null) {
            val distance = (intersection - position).length()
            if (distance < minDistance) {
                minDistance = distance
                closestIntersection = intersection
            }
        }
    }

    // If we found a potential collision, calculate avoidance vector
    if (closestIntersection != null) {
        // Find the closest segment to our position
        var closestSegment: LineSegment2D? = null
        minDistance = Double.MAX_VALUE

        for (segment in env.segments) {
            val closestPoint = segment.closestPoint(position)
            val distance = (closestPoint - position).length()
            if (distance < minDistance) {
                minDistance = distance
                closestSegment = segment
            }
        }

        // Calculate normal vector to the closest segment (pointing away from it)
        if (closestSegment != null) {
            val segmentDirection = closestSegment.direction()
            val normal = segmentDirection.normalVector()

            // Make sure the normal points away from the segment
            val toPosition = position - closestSegment.closestPoint(position)
            val dotProduct = normal.dot(toPosition)

            // Return normalized avoidance vector
            return if (dotProduct >= 0) normal else -normal
        }
    }

    return Vec2D.ZERO // No obstacles detected
}
