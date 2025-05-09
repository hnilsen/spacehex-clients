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

fun calculateAcceleration(env: Environment, lander: Lander): Acceleration {
    // Constants for our algorithm
    val distanceToSlowDown = 80.0 // Distance at which we start slowing down (further reduced for even later slowdown)
    val hoverDistance = 5.0 // Distance to goal to be considered "at goal" (as per requirements)
    val maxSpeed = 2.0 // Maximum speed allowed at goal (as per requirements)
    val safetyDistance = 20.0 // Minimum distance to keep from ground (further reduced for even faster navigation)

    // Calculate distance to goal
    val distanceToGoal = (env.goal - lander.position).length()

    // Calculate current speed
    val currentSpeed = lander.velocity.length()

    // Check if we're at the goal
    if (distanceToGoal <= hoverDistance && currentSpeed <= maxSpeed) {
        // We're at the goal and moving slowly enough, hover
        return calculateHoverAcceleration(env, lander)
    }

    // Check if we're approaching the goal and need to slow down
    if (distanceToGoal <= distanceToSlowDown) {
        // We're close to the goal, slow down
        return calculateSlowDownAcceleration(env, lander)
    }

    // We're far from the goal, navigate towards it while avoiding obstacles
    return calculateNavigationAcceleration(env, lander, safetyDistance)
}

/**
 * Calculate acceleration to hover at the goal
 */
fun calculateHoverAcceleration(env: Environment, lander: Lander): Acceleration {
    // To hover, we need to counteract gravity
    // If we're falling, accelerate up
    // If we're moving horizontally, counteract that movement

    // Calculate vertical control
    // If we're below the goal, accelerate up
    // If we're above the goal and falling slowly, don't accelerate up (let gravity work)
    // If we're above the goal and falling too fast, accelerate up to slow down
    // Adjusted threshold for more precise hovering
    val fallingTooFast = lander.velocity.y < -0.5
    val up = lander.position.y < env.goal.y || fallingTooFast

    // Calculate horizontal control
    // If we're to the right of the goal, accelerate left
    // If we're to the left of the goal, accelerate right
    // Also consider velocity to avoid overshooting
    // Adjusted thresholds for more precise hovering
    val movingRightTooFast = lander.velocity.x > 0.3
    val movingLeftTooFast = lander.velocity.x < -0.3
    val left = (lander.position.x > env.goal.x && !movingLeftTooFast) || movingRightTooFast
    val right = (lander.position.x < env.goal.x && !movingRightTooFast) || movingLeftTooFast

    return Acceleration(up = up, left = left, right = right)
}

/**
 * Calculate acceleration to slow down when approaching the goal
 */
fun calculateSlowDownAcceleration(env: Environment, lander: Lander): Acceleration {
    // Calculate direction to goal
    val directionToGoal = (env.goal - lander.position).unit()

    // Calculate dot product to see if we're moving towards or away from the goal
    val dotProduct = directionToGoal.dot(lander.velocity.unit())

    // Calculate distance to goal
    val distanceToGoal = (env.goal - lander.position).length()

    // Calculate desired maximum speed based on distance to goal
    // The closer we are to the goal, the slower we want to go
    // Further increased speed factor to allow even faster approach
    val desiredMaxSpeed = Math.max(2.0, distanceToGoal / 3.0)

    // Calculate current speed
    val currentSpeed = lander.velocity.length()

    // If we're moving too fast for our distance to the goal, slow down
    val needToSlowDown = currentSpeed > desiredMaxSpeed

    // Calculate vertical control
    // If we're falling, accelerate up
    // If we're moving up too fast and above the goal, don't accelerate up
    // Decreased threshold to allow faster vertical movement
    val fallingTooFast = lander.velocity.y < -1.5
    val movingUpTooFast = lander.velocity.y > 1.5 && lander.position.y > env.goal.y
    val up = fallingTooFast || (lander.position.y < env.goal.y && !movingUpTooFast)

    // Calculate horizontal control
    // If we're moving too fast horizontally, slow down
    // If we're moving away from the goal, reverse direction
    val movingRightTooFast = lander.velocity.x > desiredMaxSpeed / 2.0
    val movingLeftTooFast = lander.velocity.x < -desiredMaxSpeed / 2.0
    val movingAwayFromGoal = dotProduct < 0
    val left = movingRightTooFast || (movingAwayFromGoal && lander.position.x < env.goal.x)
    val right = movingLeftTooFast || (movingAwayFromGoal && lander.position.x > env.goal.x)

    return Acceleration(up = up, left = left, right = right)
}

/**
 * Calculate acceleration for navigating towards the goal while avoiding obstacles
 */
fun calculateNavigationAcceleration(env: Environment, lander: Lander, safetyDistance: Double): Acceleration {
    // Calculate direction to goal
    val directionToGoal = (env.goal - lander.position).unit()

    // Calculate distance to goal
    val distanceToGoal = (env.goal - lander.position).length()

    // Calculate desired maximum speed based on distance to goal and safety
    // The closer we are to the goal or obstacles, the slower we want to go
    // Further increased maximum speed and speed factor for even faster navigation
    val desiredMaxSpeed = Math.min(30.0, distanceToGoal / 2.0)

    // Check for potential collisions with ground segments
    val potentialCollision = checkForPotentialCollision(env, lander, safetyDistance)

    if (potentialCollision) {
        // If there's a potential collision, navigate away from the ground
        return calculateAvoidanceAcceleration(env, lander, safetyDistance)
    }

    // Calculate current speed
    val currentSpeed = lander.velocity.length()

    // Calculate vertical control
    // If we're moving in the direction of the goal, maintain that direction
    // If we're falling too fast, accelerate up
    // If we need to move up to reach the goal, accelerate up
    // Decreased threshold to allow faster vertical movement
    val fallingTooFast = lander.velocity.y < -3.0
    val needToMoveUp = directionToGoal.y > 0.2
    val movingUpTooFast = lander.velocity.y > desiredMaxSpeed && !needToMoveUp
    val up = fallingTooFast || (needToMoveUp && !movingUpTooFast)

    // Calculate horizontal control
    // If we need to move left to reach the goal, accelerate left (unless already moving left too fast)
    // If we need to move right to reach the goal, accelerate right (unless already moving right too fast)
    val needToMoveLeft = directionToGoal.x < -0.2
    val needToMoveRight = directionToGoal.x > 0.2
    val movingLeftTooFast = lander.velocity.x < -desiredMaxSpeed
    val movingRightTooFast = lander.velocity.x > desiredMaxSpeed
    val left = (needToMoveLeft && !movingLeftTooFast) || movingRightTooFast
    val right = (needToMoveRight && !movingRightTooFast) || movingLeftTooFast

    return Acceleration(up = up, left = left, right = right)
}

/**
 * Check if there's a potential collision with the ground
 */
fun checkForPotentialCollision(env: Environment, lander: Lander, safetyDistance: Double): Boolean {
    // Project the lander's position forward based on its velocity
    // Check multiple points along the trajectory for collision detection
    // Further reduced number of points for even less cautious navigation
    val numPoints = 2
    val velocityStep = lander.velocity.div(numPoints.toDouble())

    // Check distance to each ground segment
    for (segment in env.segments) {
        // Check current position
        val closestPoint = segment.closestPoint(lander.position)
        val distance = (closestPoint - lander.position).length()

        if (distance < safetyDistance) {
            return true
        }

        // Check multiple projected positions along the trajectory
        var projectedPosition = lander.position
        for (i in 1..numPoints) {
            projectedPosition = projectedPosition + velocityStep
            val projectedClosestPoint = segment.closestPoint(projectedPosition)
            val projectedDistance = (projectedClosestPoint - projectedPosition).length()

            if (projectedDistance < safetyDistance) {
                return true
            }
        }

        // Also check if the lander's trajectory intersects with the segment
        val trajectory = LineSegment2D(lander.position, lander.position + lander.velocity)
        if (trajectory.intersects(segment) != null) {
            return true
        }
    }

    return false
}

/**
 * Calculate acceleration to avoid collision with the ground
 */
fun calculateAvoidanceAcceleration(env: Environment, lander: Lander, safetyDistance: Double): Acceleration {
    // Find the closest ground segment
    var closestSegment: LineSegment2D? = null
    var minDistance = Double.MAX_VALUE
    var closestPoint = Vec2D(0.0, 0.0)

    for (segment in env.segments) {
        val point = segment.closestPoint(lander.position)
        val distance = (point - lander.position).length()

        if (distance < minDistance) {
            minDistance = distance
            closestSegment = segment
            closestPoint = point
        }
    }

    // If we found a close segment, move away from it
    if (closestSegment != null) {
        // Calculate avoidance direction (away from the closest point)
        val avoidanceDirection = (lander.position - closestPoint).unit()

        // Calculate projected position based on current velocity
        val projectedPosition = lander.position + lander.velocity

        // Calculate if the projected position is closer to the ground
        val projectedClosestPoint = closestSegment.closestPoint(projectedPosition)
        val projectedDistance = (projectedClosestPoint - projectedPosition).length()
        val gettingCloser = projectedDistance < minDistance

        // Calculate dot product between velocity and avoidance direction
        // If positive, we're already moving away from the ground
        // If negative, we're moving towards the ground
        val dotProduct = avoidanceDirection.dot(lander.velocity.unit())
        val movingTowardsGround = dotProduct < 0

        // Calculate vertical control
        // If avoidance direction is up, or we're falling, or we're getting closer to the ground, accelerate up
        val needToMoveUp = avoidanceDirection.y > 0.2
        // Decreased threshold to allow faster vertical movement
        val fallingTooFast = lander.velocity.y < -1.5
        val up = needToMoveUp || fallingTooFast || (gettingCloser && movingTowardsGround)

        // Calculate horizontal control
        // If avoidance direction is left/right, and we're not already moving too fast in that direction, accelerate in that direction
        val needToMoveLeft = avoidanceDirection.x < -0.2
        val needToMoveRight = avoidanceDirection.x > 0.2
        // Further increased speed thresholds for even faster avoidance
        val movingLeftTooFast = lander.velocity.x < -12.0
        val movingRightTooFast = lander.velocity.x > 12.0
        val left = (needToMoveLeft && !movingLeftTooFast) || (movingRightTooFast && gettingCloser)
        val right = (needToMoveRight && !movingRightTooFast) || (movingLeftTooFast && gettingCloser)

        return Acceleration(up = up, left = left, right = right)
    }

    // Default to moving up if we can't determine a better direction
    return Acceleration(up = true, left = false, right = false)
}
