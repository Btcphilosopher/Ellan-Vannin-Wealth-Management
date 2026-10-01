using LinearAlgebra
using Statistics

# ============================================================
# URBAN AUTONOMOUS PARKING OPTIMISER
# Julia prototype
# ============================================================

struct Pose
    x::Float64
    y::Float64
    θ::Float64
end

struct Vehicle
    length::Float64
    width::Float64
    wheelbase::Float64
    max_steering::Float64
    max_acceleration::Float64
    max_deceleration::Float64
end

struct ParkingSpace
    x::Float64
    y::Float64
    length::Float64
    width::Float64
    heading::Float64
end

struct Obstacle
    x::Float64
    y::Float64
    radius::Float64
end

struct Trajectory
    poses::Vector{Pose}
    steering::Vector{Float64}
    speed::Vector{Float64}
    cost::Float64
end


# ============================================================
# VEHICLE
# ============================================================

car = Vehicle(
    4.7,              # length
    1.85,             # width
    2.8,              # wheelbase
    deg2rad(32),      # max steering
    2.0,              # acceleration
    5.0               # braking
)


# ============================================================
# PARKING SPACE SCORING
# ============================================================

function parking_space_score(
    space::ParkingSpace,
    vehicle::Vehicle
)

    length_margin =
        space.length - vehicle.length

    width_margin =
        space.width - vehicle.width

    # Reject spaces that are physically impossible
    if length_margin < 0.4 ||
       width_margin < 0.25

        return Inf
    end

    # Prefer larger spaces
    clearance_cost =
        1.0 /
        (length_margin * width_margin)

    return clearance_cost
end


# ============================================================
# COLLISION MODEL
# ============================================================

function collision(
    pose::Pose,
    vehicle::Vehicle,
    obstacles::Vector{Obstacle}
)

    half_length = vehicle.length / 2
    half_width  = vehicle.width / 2

    # Conservative circular approximation
    safety_radius =
        sqrt(
            half_length^2 +
            half_width^2
        )

    for obstacle in obstacles

        distance =
            hypot(
                pose.x - obstacle.x,
                pose.y - obstacle.y
            )

        if distance <
           safety_radius +
           obstacle.radius

            return true
        end
    end

    return false
end


# ============================================================
# KINEMATIC VEHICLE MODEL
# ============================================================

function propagate(
    pose::Pose,
    steering::Float64,
    velocity::Float64,
    dt::Float64,
    vehicle::Vehicle
)

    x = pose.x +
        velocity *
        cos(pose.θ) *
        dt

    y = pose.y +
        velocity *
        sin(pose.θ) *
        dt

    θ = pose.θ +
        velocity /
        vehicle.wheelbase *
        tan(steering) *
        dt

    return Pose(x, y, θ)
end


# ============================================================
# TRAJECTORY GENERATOR
# ============================================================

function generate_trajectory(
    start::Pose,
    controls::Vector{Tuple{Float64,Float64}},
    vehicle::Vehicle,
    obstacles::Vector{Obstacle}
)

    poses = Pose[start]

    steering_history = Float64[]
    speed_history = Float64[]

    current = start

    for (steering, speed) in controls

        next =
            propagate(
                current,
                steering,
                speed,
                0.1,
                vehicle
            )

        if collision(
            next,
            vehicle,
            obstacles
        )

            return nothing
        end

        push!(
            poses,
            next
        )

        push!(
            steering_history,
            steering
        )

        push!(
            speed_history,
            speed
        )

        current = next
    end

    return poses,
           steering_history,
           speed_history
end


# ============================================================
# PARKING TRAJECTORY COST
# ============================================================

function trajectory_cost(
    poses::Vector{Pose},
    steering::Vector{Float64},
    speed::Vector{Float64},
    target::Pose
)

    # Final position error
    final = poses[end]

    position_error =
        hypot(
            final.x - target.x,
            final.y - target.y
        )

    heading_error =
        abs(
            atan(
                sin(final.θ - target.θ),
                cos(final.θ - target.θ)
            )
        )

    # Smoothness
    steering_cost =
        sum(
            abs.(diff(steering))
        )

    # Excessive speed is undesirable
    speed_cost =
        sum(
            abs.(speed)
        )

    # Reverse/forward manoeuvres cost time
    manoeuvre_cost =
        length(poses)

    return (
        100.0 * position_error +
        40.0  * heading_error +
        2.0   * steering_cost +
        0.2   * speed_cost +
        0.1   * manoeuvre_cost
    )
end


# ============================================================
# CONTROL SEARCH
# ============================================================

function optimise_parking(
    start::Pose,
    target::Pose,
    vehicle::Vehicle,
    obstacles::Vector{Obstacle}
)

    best = nothing
    best_cost = Inf

    steering_values =
        range(
            -vehicle.max_steering,
            vehicle.max_steering,
            length= nine()
        )

    speed_values =
        [-1.5, -0.75, 0.0, 0.75, 1.5]

    # --------------------------------------------------------
    # Simplified model-predictive search
    # --------------------------------------------------------

    for steering in steering_values

        for speed in speed_values

            controls =
                [
                    (steering, speed)
                    for _ in 1:40
                ]

            result =
                generate_trajectory(
                    start,
                    controls,
                    vehicle,
                    obstacles
                )

            result === nothing &&
                continue

            poses,
            steering_history,
            speed_history = result

            cost =
                trajectory_cost(
                    poses,
                    steering_history,
                    speed_history,
                    target
                )

            if cost < best_cost

                best_cost = cost

                best =
                    Trajectory(
                        poses,
                        steering_history,
                        speed_history,
                        cost
                    )
            end
        end
    end

    return best
end


# ============================================================
# PARKING TARGET
# ============================================================

function parking_target(
    space::ParkingSpace
)

    return Pose(
        space.x,
        space.y,
        space.heading
    )
end


# ============================================================
# EXAMPLE
# ============================================================

start =
    Pose(
        0.0,
        0.0,
        0.0
    )

space =
    ParkingSpace(
        8.0,
        1.0,
        5.5,
        2.4,
        0.0
    )

target =
    parking_target(space)

obstacles = [
    Obstacle(5.0, 2.0, 0.5),
    Obstacle(6.0, -1.0, 0.5),
    Obstacle(9.0, 2.0, 0.5)
]

trajectory =
    optimise_parking(
        start,
        target,
        car,
        obstacles
    )

if trajectory === nothing

    println(
        "No safe parking trajectory found."
    )

else

    println(
        "Parking trajectory found."
    )

    println(
        "Trajectory cost = ",
        trajectory.cost
    )

    println(
        "Trajectory points = ",
        length(trajectory.poses)
    )

end
