R — Predictive Cornering / Steering Executor
# ============================================================
# PREDICTIVE CORNER EXECUTOR
# R VEHICLE DYNAMICS / CONTROL PROTOTYPE
#
# Road preview
#      |
#      v
# Corner detection
#      |
#      v
# Curvature estimation
#      |
#      v
# Corner speed prediction
#      |
#      v
# Steering feed-forward
#      |
#      v
# Vehicle-state correction
#      |
#      v
# Steering execution
#      |
#      v
# Corner exit / steering unwind
#
# Research / simulation implementation.
# ============================================================


# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------

corner_config <- list(

  wheelbase = 2.85,
  mass = 1850,

  max_steering_angle = 32 * pi / 180,
  max_steering_rate = 90 * pi / 180,

  max_lateral_accel = 8.0,

  # Conservative fraction of estimated lateral limit
  safety_factor = 0.80,

  preview_distance = 100,

  steering_gain = 0.25,
  curvature_gain = 0.15,
  yaw_gain = 0.20,

  entry_margin = 2.0,
  exit_margin = 1.0,

  dt = 0.02
)


# ------------------------------------------------------------
# Vehicle state
# ------------------------------------------------------------

vehicle_state <- list(

  speed = 27.0,

  steering_angle = 0.0,
  steering_rate = 0.0,

  yaw_rate = 0.0,

  lateral_accel = 0.0,

  lateral_error = 0.0,
  heading_error = 0.0
)


# ------------------------------------------------------------
# Road preview
#
# curvature = 1 / radius
# Positive = left
# Negative = right
# ------------------------------------------------------------

road_preview <- list(

  distance = c(
    0, 20, 40, 60, 80, 100
  ),

  curvature = c(
    0.000,
    0.003,
    0.008,
    0.015,
    0.020,
    0.010
  )
)


# ------------------------------------------------------------
# Clamp
# ------------------------------------------------------------

clamp <- function(x, minimum, maximum) {

  max(minimum, min(x, maximum))

}


# ------------------------------------------------------------
# Detect corner
# ------------------------------------------------------------

detect_corner <- function(preview) {

  curvature <- abs(preview$curvature)

  if (max(curvature) < 1e-4) {
    return(FALSE)
  }

  TRUE
}


# ------------------------------------------------------------
# Estimate corner radius
# ------------------------------------------------------------

estimate_corner_radius <- function(preview) {

  curvature <- abs(preview$curvature)

  valid <- curvature > 1e-5

  if (!any(valid)) {
    return(Inf)
  }

  maximum_curvature <- max(curvature[valid])

  1 / maximum_curvature
}


# ------------------------------------------------------------
# Corner severity
# ------------------------------------------------------------

corner_severity <- function(preview) {

  maximum_curvature <-
    max(abs(preview$curvature))

  severity <-
    maximum_curvature / 0.04

  clamp(
    severity,
    0,
    1
  )
}


# ------------------------------------------------------------
# Corner speed
#
# a_y = v^2 * k
#
# therefore:
#
# v = sqrt(a_y / k)
# ------------------------------------------------------------

corner_speed <- function(config, curvature) {

  if (abs(curvature) < 1e-6) {
    return(Inf)
  }

  allowed_lateral_accel <-
    config$max_lateral_accel *
    config$safety_factor

  sqrt(
    allowed_lateral_accel /
      abs(curvature)
  )
}


# ------------------------------------------------------------
# Predict target speed
# ------------------------------------------------------------

target_corner_speed <- function(
    config,
    state,
    preview
) {

  curvature <-
    max(abs(preview$curvature))

  speed_limit <-
    corner_speed(
      config,
      curvature
    )

  target <-
    speed_limit -
    config$entry_margin

  max(
    target,
    5.0
  )
}


# ------------------------------------------------------------
# Curvature -> steering
#
# Bicycle-model approximation:
#
# delta = atan(L * curvature)
# ------------------------------------------------------------

curvature_steering <- function(
    config,
    curvature
) {

  steering <-
    atan(
      config$wheelbase *
        curvature
    )

  clamp(
    steering,
    -config$max_steering_angle,
    config$max_steering_angle
  )
}


# ------------------------------------------------------------
# Predictive steering
# ------------------------------------------------------------

predictive_steering <- function(
    config,
    state,
    preview
) {

  # Select strongest upcoming curvature
  index <-
    which.max(
      abs(preview$curvature)
    )

  curvature <-
    preview$curvature[index]


  # -------------------------------
  # 1. Feed-forward steering
  # -------------------------------

  feedforward <-
    curvature_steering(
      config,
      curvature
    )


  # -------------------------------
  # 2. Lateral position correction
  # -------------------------------

  lateral_correction <-
    -config$steering_gain *
    state$lateral_error


  # -------------------------------
  # 3. Heading correction
  # -------------------------------

  heading_correction <-
    -config$yaw_gain *
    state$heading_error


  # -------------------------------
  # 4. Yaw damping
  # -------------------------------

  yaw_correction <-
    -config$curvature_gain *
    state$yaw_rate


  steering <-
    feedforward +
    lateral_correction +
    heading_correction +
    yaw_correction


  clamp(
    steering,
    -config$max_steering_angle,
    config$max_steering_angle
  )
}


# ------------------------------------------------------------
# Steering rate limiter
# ------------------------------------------------------------

limit_steering_rate <- function(
    config,
    current_angle,
    target_angle
) {

  maximum_change <-
    config$max_steering_rate *
    config$dt

  difference <-
    target_angle -
    current_angle

  difference <-
    clamp(
      difference,
      -maximum_change,
      maximum_change
    )

  current_angle +
    difference
}


# ------------------------------------------------------------
# Corner phase
# ------------------------------------------------------------

corner_phase <- function(preview) {

  curvature <-
    abs(preview$curvature)

  n <- length(curvature)

  if (n < 3) {
    return("entry")
  }


  midpoint <-
    floor(n / 2)


  first_half <-
    mean(
      curvature[
        1:midpoint
      ]
    )


  second_half <-
    mean(
      curvature[
        (midpoint + 1):n
      ]
    )


  peak <-
    which.max(curvature)


  if (peak <= n / 3) {

    return("entry")

  }


  if (peak >= 2 * n / 3) {

    return("exit")

  }


  if (second_half >
      first_half * 0.9) {

    return("apex")

  }


  "exit"
}


# ------------------------------------------------------------
# Main corner executor
# ------------------------------------------------------------

execute_corner <- function(
    config,
    state,
    preview
) {

  detected <-
    detect_corner(preview)


  if (!detected) {

    return(list(

      target_speed = Inf,

      steering_angle = 0,

      corner_detected = FALSE,

      corner_radius = Inf,

      corner_severity = 0,

      phase = "straight"

    ))
  }


  radius <-
    estimate_corner_radius(
      preview
    )


  severity <-
    corner_severity(
      preview
    )


  target_speed <-
    target_corner_speed(
      config,
      state,
      preview
    )


  desired_steering <-
    predictive_steering(
      config,
      state,
      preview
    )


  steering <-
    limit_steering_rate(
      config,
      state$steering_angle,
      desired_steering
    )


  phase <-
    corner_phase(preview)


  # Gradually unwind steering
  # during corner exit.

  if (phase == "exit") {

    steering <-
      steering * 0.75
  }


  list(

    target_speed = target_speed,

    steering_angle = steering,

    corner_detected = TRUE,

    corner_radius = radius,

    corner_severity = severity,

    phase = phase

  )
}


# ------------------------------------------------------------
# EXECUTE
# ------------------------------------------------------------

command <- execute_corner(
  corner_config,
  vehicle_state,
  road_preview
)


cat(
  "Corner detected:",
  command$corner_detected,
  "\n"
)

cat(
  "Corner radius:",
  round(command$corner_radius, 1),
  "m\n"
)

cat(
  "Corner severity:",
  round(command$corner_severity, 2),
  "\n"
)

cat(
  "Corner phase:",
  command$phase,
  "\n"
)

cat(
  "Target speed:",
  round(command$target_speed, 2),
  "m/s\n"
)

cat(
  "Target speed:",
  round(command$target_speed * 3.6, 1),
  "km/h\n"
)

cat(
  "Steering:",
  round(
    command$steering_angle * 180 / pi,
    2
  ),
  "degrees\n"
)
The important part

The controller isn't simply saying:

"The road is curving, turn the wheel."

It's effectively doing:

100 m
 │
 │  ROAD PREVIEW
 │
 ▼
───────────────────────────────╮
                               │
                        CURVE  │
                         ╭─────╯
                         │
                         │
                         ╰────────────
 
        ↓
    predict κ(s)

        ↓
    predict radius

        ↓
    calculate lateral-force requirement

        ↓
    calculate target entry speed

        ↓
    calculate steering feed-forward

        ↓
    correct using actual yaw/lateral error

        ↓
    progressively unwind steering

That speed–curvature coupling is particularly important. Research on predictive cornering controllers explicitly uses future curvature to slow the vehicle before entering the corner rather than waiting until lateral acceleration has already increased.

R version with a proper corner trajectory

For your larger vehicle-AI project, I'd next replace the single "strongest curvature" calculation with a curvature profile:

corner_prediction <- list(

  distance = c(
    100, 90, 80, 70, 60, 50,
    40, 30, 20, 10, 0
  ),

  curvature = c(
    0.000, 0.002, 0.004, 0.007,
    0.011, 0.016, 0.020, 0.021,
    0.018, 0.012, 0.005
  )

)

R can then calculate the entire trajectory:

                 CORNER PLAN

ENTRY                         APEX                    EXIT

100m      70m       50m       30m       10m       0m
 │         │         │         │         │         │
 ▼         ▼         ▼         ▼         ▼         ▼

105 ───── 100 ───── 96 ───── 86 ───── 82 ───── 88 km/h
     ↓       ↓        ↓         ↓         ↑
   brake   brake    rotate    hold      unwind
                         \
                          \
                    steering builds
                          │
                          ▼
                     maximum turn
                          │
                          ▼
                    steering unwind

Then the R optimiser can choose a complete sequence of speed + steering rather than one target:

$$ J = w_1 e_\text{path}^2 + w_2 e_\text{speed}^2 + w_3 e_\text{steering}^2 + w_4 \dot{\delta}^2 + w_5 a_y^2 + w_6 j^2 $$

with physical constraints on steering, steering rate, acceleration, lateral acceleration and available tyre force. This is much closer to a genuine MPC architecture; published work has specifically used constrained MPC for cornering performance and handling characteristics.

And because you've already built the R/Julia weight-distribution model, the next logical step is to feed predicted FL/FR/RL/RR wheel loads into this corner optimiser. That lets the algorithm predict not just where the road turns, but how the car's mass will transfer as it turns, and adjust the steering/braking trajectory accordingly. Research on curvature/friction-preview MPC similarly treats longitudinal and lateral forces as coupled constraints rather than independent controls.

