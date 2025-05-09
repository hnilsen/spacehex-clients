package no.sonat.game

import no.sonat.game.geometry.LineSegment2D
import no.sonat.game.geometry.Vec2D
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import kotlin.math.min

/**
 * Helper class for terrain analysis and path planning.
 * Provides functions to analyze the terrain, find safe paths, and generate
 * sequential commands for the lander to navigate to the goal.
 */
class TerrainNavigator(
    private val environment: Environment,
    private val config: LanderConfig = LanderConfig()
) {
    private val logger: Logger = LoggerFactory.getLogger(TerrainNavigator::class.java)
    
    // The server operates on 0.1 second refresh rate
    private val timeStep = environment.constants.timeDeltaSeconds
    
    /**
     * Analyzes the terrain and identifies safe areas (air) and dangerous areas (ground).
     * @param currentPosition The current position of the lander
     * @param radius The radius around the current position to analyze
     * @return A list of safe points that can be used for path planning
     */
    fun analyzeTerrain(currentPosition: Vec2D, radius: Double = 100.0): List<Vec2D> {
        val safePoints = mutableListOf<Vec2D>()
        
        // Create a grid of points to check
        val gridSize = 20.0
        val xMin = currentPosition.x - radius
        val xMax = currentPosition.x + radius
        val yMin = currentPosition.y - radius
        val yMax = currentPosition.y + radius
        
        for (x in generateSequence(xMin) { it + gridSize }.takeWhile { it <= xMax }) {
            for (y in generateSequence(yMin) { it + gridSize }.takeWhile { it <= yMax }) {
                val point = Vec2D(x, y)
                
                // Check if the point is safe (not too close to ground)
                if (isPointSafe(point)) {
                    safePoints.add(point)
                }
            }
        }
        
        // Always include the goal if it's safe
        if (isPointSafe(environment.goal)) {
            safePoints.add(environment.goal)
        }
        
        return safePoints
    }
    
    /**
     * Checks if a point is safe (not too close to ground).
     * @param point The point to check
     * @param safeDistance The minimum safe distance from ground
     * @return True if the point is safe, false otherwise
     */
    fun isPointSafe(point: Vec2D, safeDistance: Double = 15.0): Boolean {
        // Check if the point is within the game boundaries
        if (point.x < -512 || point.x > 512 || point.y < -384 || point.y > 384) {
            return false
        }
        
        // Check distance to each ground segment
        for (segment in environment.segments) {
            val closestPoint = segment.closestPoint(point)
            val distance = (point - closestPoint).length()
            
            if (distance < safeDistance) {
                return false
            }
        }
        
        return true
    }
    
    /**
     * Finds a safe path from the current position to the goal.
     * Uses a simplified A* algorithm to find a path through safe points.
     * @param currentPosition The current position of the lander
     * @param currentVelocity The current velocity of the lander
     * @return A list of waypoints forming a path to the goal
     */
    fun findPath(currentPosition: Vec2D, currentVelocity: Vec2D): List<Vec2D> {
        // If we're already close to the goal and it's safe, go directly there
        val distanceToGoal = (environment.goal - currentPosition).length()
        if (distanceToGoal < 50.0 && isPathSafe(currentPosition, environment.goal)) {
            return listOf(environment.goal)
        }
        
        // Get safe points for path planning
        val safePoints = analyzeTerrain(currentPosition)
        if (safePoints.isEmpty()) {
            logger.warn("No safe points found for path planning!")
            return listOf(environment.goal) // Fallback to direct path
        }
        
        // Find the best path through safe points
        val path = mutableListOf<Vec2D>()
        path.add(currentPosition)
        
        // Simple greedy algorithm: always choose the safe point that gets us closest to the goal
        var current = currentPosition
        val maxIterations = 10 // Limit the number of waypoints
        
        for (i in 0 until maxIterations) {
            // Find the best next point
            val nextPoint = findBestNextPoint(current, currentVelocity, safePoints)
            
            // If we couldn't find a next point or we're close to the goal, stop
            if (nextPoint == null || (environment.goal - current).length() < 20.0) {
                break
            }
            
            path.add(nextPoint)
            current = nextPoint
            
            // If we can go directly to the goal from here, add it and stop
            if (isPathSafe(current, environment.goal)) {
                path.add(environment.goal)
                break
            }
        }
        
        // Make sure the goal is the last point
        if (path.last() != environment.goal) {
            path.add(environment.goal)
        }
        
        logger.info("Path found with ${path.size} waypoints: $path")
        return path
    }
    
    /**
     * Finds the best next point to move to from the current position.
     * @param currentPosition The current position
     * @param currentVelocity The current velocity
     * @param safePoints List of safe points to choose from
     * @return The best next point, or null if none found
     */
    private fun findBestNextPoint(
        currentPosition: Vec2D, 
        currentVelocity: Vec2D, 
        safePoints: List<Vec2D>
    ): Vec2D? {
        var bestPoint: Vec2D? = null
        var bestScore = Double.MAX_VALUE
        
        for (point in safePoints) {
            // Skip the current position
            if (point == currentPosition) continue
            
            // Check if we can safely move to this point
            if (!isPathSafe(currentPosition, point)) continue
            
            // Calculate score based on:
            // 1. Distance to goal (lower is better)
            // 2. Alignment with current velocity (higher is better)
            // 3. Distance from current position (not too far, not too close)
            val distanceToGoal = (environment.goal - point).length()
            
            val directionToPoint = (point - currentPosition).unit()
            val velocityAlignment = if (currentVelocity.length() > 0.1) {
                currentVelocity.unit().dot(directionToPoint)
            } else {
                0.0
            }
            
            val distanceFromCurrent = (point - currentPosition).length()
            val idealDistance = 50.0 // Not too far, not too close
            val distanceScore = Math.abs(distanceFromCurrent - idealDistance)
            
            // Combined score (lower is better)
            val score = distanceToGoal * 2.0 - velocityAlignment * 50.0 + distanceScore
            
            if (score < bestScore) {
                bestScore = score
                bestPoint = point
            }
        }
        
        return bestPoint
    }
    
    /**
     * Checks if a direct path between two points is safe (doesn't intersect with ground).
     * @param from Starting point
     * @param to Ending point
     * @return True if the path is safe, false otherwise
     */
    fun isPathSafe(from: Vec2D, to: Vec2D): Boolean {
        val path = LineSegment2D(from, to)
        
        // Check for intersections with ground segments
        for (segment in environment.segments) {
            if (path.intersects(segment) != null) {
                return false
            }
        }
        
        // Also check a few points along the path
        val steps = 5
        for (i in 1 until steps) {
            val t = i.toDouble() / steps
            val point = from + (to - from) * t
            if (!isPointSafe(point)) {
                return false
            }
        }
        
        return true
    }
    
    /**
     * Calculates the sequence of acceleration commands needed to follow a path.
     * @param lander The current lander state
     * @param path The path to follow
     * @return The acceleration command to apply now
     */
    fun calculatePathAcceleration(lander: Lander, path: List<Vec2D>): Acceleration {
        // If we've already reached the goal or crashed, don't do anything
        if (lander.status != LanderStatus.FLYING) {
            return Acceleration(up = false, left = false, right = false)
        }
        
        // If the path is empty, use the goal directly
        if (path.isEmpty()) {
            return calculateAccelerationToPoint(lander, environment.goal)
        }
        
        // Find the next waypoint to target
        val nextWaypoint = findNextWaypoint(lander.position, path)
        
        // Calculate acceleration to reach the next waypoint
        return calculateAccelerationToPoint(lander, nextWaypoint)
    }
    
    /**
     * Finds the next waypoint to target from the path.
     * @param currentPosition The current position
     * @param path The path to follow
     * @return The next waypoint to target
     */
    private fun findNextWaypoint(currentPosition: Vec2D, path: List<Vec2D>): Vec2D {
        // If we're close to the first waypoint, target the second one (if available)
        if (path.size > 1) {
            val distanceToFirst = (path[0] - currentPosition).length()
            if (distanceToFirst < 20.0) {
                return path[1]
            }
        }
        
        // Otherwise, target the first waypoint
        return path[0]
    }
    
    /**
     * Calculates the acceleration needed to reach a specific point.
     * @param lander The current lander state
     * @param targetPoint The point to reach
     * @return The acceleration command to apply
     */
    fun calculateAccelerationToPoint(lander: Lander, targetPoint: Vec2D): Acceleration {
        // Calculate vector to target
        val toTarget = targetPoint - lander.position
        val distanceToTarget = toTarget.length()
        
        // Calculate current speed
        val currentSpeed = lander.velocity.length()
        
        // Calculate desired speed based on distance to target
        val desiredSpeed = when {
            distanceToTarget > config.slowdownDistance -> config.maxSpeed
            distanceToTarget < config.nearGoalThreshold -> min(2.0, distanceToTarget * config.nearGoalSpeedFactor)
            else -> config.maxSpeed * (distanceToTarget / config.slowdownDistance)
        }
        
        // Calculate desired velocity vector
        val desiredVelocity = if (distanceToTarget > config.veryCloseToGoalThreshold) {
            toTarget.unit() * desiredSpeed
        } else {
            Vec2D(0.0, 0.0) // If very close, aim for zero velocity
        }
        
        // Calculate velocity difference
        val velocityDiff = desiredVelocity - lander.velocity
        
        // Check if we're about to hit the ground
        val groundCollisionImminent = isGroundCollisionImminent(lander)
        
        // Determine acceleration based on velocity difference and collision avoidance
        val needUp = (velocityDiff.y > config.verticalAccelerationThreshold) || 
                     groundCollisionImminent || 
                     (distanceToTarget < config.approachingGoalThreshold && lander.velocity.y < config.fallingTooFastThreshold) || 
                     (lander.position.y < config.bottomScreenThreshold)
        
        // Adjust horizontal acceleration thresholds based on distance to target
        val horizontalThreshold = if (distanceToTarget < config.nearGoalThreshold) {
            config.nearGoalHorizontalThreshold
        } else {
            config.farGoalHorizontalThreshold
        }
        
        val needLeft = velocityDiff.x < -horizontalThreshold || 
                      (distanceToTarget < config.nearGoalThreshold && lander.velocity.x > config.hoverHorizontalVelocityThreshold)
        
        val needRight = velocityDiff.x > horizontalThreshold || 
                       (distanceToTarget < config.nearGoalThreshold && lander.velocity.x < -config.hoverHorizontalVelocityThreshold)
        
        // Apply acceleration
        return Acceleration(
            up = needUp,
            left = needLeft,
            right = needRight
        )
    }
    
    /**
     * Checks if the lander is about to collide with the ground.
     * @param lander The current lander state
     * @return True if collision is imminent, false otherwise
     */
    private fun isGroundCollisionImminent(lander: Lander): Boolean {
        // Calculate current speed
        val currentSpeed = lander.velocity.length()
        
        // Adjust look-ahead time based on speed
        val futureTime = min(
            config.baseLookAheadTime + currentSpeed * config.speedFactor, 
            config.maxLookAheadTime
        )
        
        val gravity = environment.constants.gravity
        
        // Calculate future position with gravity
        val futurePosition = Vec2D(
            lander.position.x + lander.velocity.x * futureTime,
            lander.position.y + lander.velocity.y * futureTime - 0.5 * gravity * futureTime * futureTime
        )
        
        // Create a line segment from current position to future position
        val trajectory = LineSegment2D(lander.position, futurePosition)
        
        // Check for intersection with any ground segment
        for (segment in environment.segments) {
            if (trajectory.intersects(segment) != null) {
                logger.info("Collision detected: trajectory intersects with ground segment")
                return true
            }
        }
        
        // Also check if the future position is very close to any ground segment
        val safeDistance = config.baseSafeDistance + currentSpeed * config.safeDistanceSpeedFactor
        
        for (segment in environment.segments) {
            val closestPoint = segment.closestPoint(futurePosition)
            val distance = (futurePosition - closestPoint).length()
            
            if (distance < safeDistance) {
                logger.info("Collision detected: future position too close to ground segment (${distance} < ${safeDistance})")
                return true
            }
        }
        
        // Check if we're heading toward the ground
        var closestDistance = Double.MAX_VALUE
        var closestSegment: LineSegment2D? = null
        
        for (segment in environment.segments) {
            val closestPoint = segment.closestPoint(lander.position)
            val distance = (lander.position - closestPoint).length()
            
            if (distance < closestDistance) {
                closestDistance = distance
                closestSegment = segment
            }
        }
        
        if (closestSegment != null && closestDistance < config.closeDistanceThreshold) {
            val velocityDirection = lander.velocity.unit()
            val toClosestPoint = (closestSegment.closestPoint(lander.position) - lander.position).unit()
            
            // If the dot product is positive, we're heading toward the closest point
            val dotProduct = velocityDirection.dot(toClosestPoint)
            if (dotProduct > config.dotProductThreshold) {
                logger.info("Collision predicted: heading toward ground (dot product: $dotProduct)")
                return true
            }
        }
        
        return false
    }
    
    /**
     * Returns the minimum of two values.
     */
    private fun min(a: Double, b: Double): Double = if (a < b) a else b
}