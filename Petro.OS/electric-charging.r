# ============================================================
# AUREOM EV FAST-CHARGING OPTIMISER
# R / BATTERY CHARGING SIMULATION
#
# Research / simulation controller.
#
# Battery
#     |
#     +---- SOC
#     +---- Temperature
#     +---- Voltage
#     +---- Current
#     |
#     v
# Thermal model
#     |
#     v
# BMS charging limits
#     |
#     +---- Battery limit
#     +---- Charger limit
#     +---- Thermal limit
#     +---- SOC taper
#     |
#     v
# Optimal charging power
#     |
#     v
# DC charger command
# ============================================================


# ------------------------------------------------------------
# Battery configuration
# ------------------------------------------------------------

battery_config <- list(

  capacity_kwh = 82.0,

  nominal_voltage = 800,

  maximum_voltage = 920,

  minimum_voltage = 600,

  maximum_current = 500,

  maximum_charge_power_kw = 350,

  maximum_soc = 0.98,

  target_soc = 0.80,

  minimum_soc = 0.05,

  # Battery temperature limits
  minimum_temperature = 5,
  optimal_temperature = 30,
  maximum_temperature = 45,

  # Thermal model
  thermal_mass = 420,

  cooling_power_kw = 35,

  thermal_resistance = 0.08,

  # Charging efficiency
  charge_efficiency = 0.96,

  # Controller timestep
  dt = 1.0
)


# ------------------------------------------------------------
# Battery state
# ------------------------------------------------------------

battery_state <- list(

  soc = 0.10,

  temperature = 24.0,

  voltage = 760,

  current = 0,

  power_kw = 0,

  energy_added_kwh = 0,

  charging = FALSE
)


# ------------------------------------------------------------
# Charger
# ------------------------------------------------------------

charger <- list(

  maximum_power_kw = 350,

  maximum_current = 500,

  voltage = 1000
)


# ------------------------------------------------------------
# Utility
# ------------------------------------------------------------

clamp <- function(
    x,
    minimum,
    maximum
) {

  max(
    minimum,
    min(x, maximum)
  )
}


# ------------------------------------------------------------
# Battery voltage estimate
#
# Simplified SOC-dependent model.
# ------------------------------------------------------------

battery_voltage <- function(
    config,
    state
) {

  soc <- state$soc

  voltage <-
    config$minimum_voltage +
    soc *
    (config$maximum_voltage -
       config$minimum_voltage)

  voltage
}


# ------------------------------------------------------------
# Temperature factor
#
# Maximum charging power occurs around
# the preferred battery temperature.
# ------------------------------------------------------------

temperature_factor <- function(
    config,
    temperature
) {

  if (
    temperature <
    config$minimum_temperature
  ) {

    return(0.20)
  }


  if (
    temperature <=
    config$optimal_temperature
  ) {

    # Gradual increase toward optimal
    return(
      0.50 +
      0.50 *
      (
        temperature -
        config$minimum_temperature
      ) /
      (
        config$optimal_temperature -
        config$minimum_temperature
      )
    )
  }


  if (
    temperature <=
    config$maximum_temperature
  ) {

    # Reduce power as battery gets hot
    return(
      1.0 -
      0.75 *
      (
        temperature -
        config$optimal_temperature
      ) /
      (
        config$maximum_temperature -
        config$optimal_temperature
      )
    )
  }


  0.0
}


# ------------------------------------------------------------
# SOC charging taper
#
# Allows high power at low SOC and progressively
# reduces power near the upper SOC region.
# ------------------------------------------------------------

soc_power_factor <- function(
    soc
) {

  if (soc < 0.50) {

    return(1.0)
  }


  if (soc < 0.70) {

    return(
      1.0 -
      0.10 *
      (soc - 0.50) /
      0.20
    )
  }


  if (soc < 0.80) {

    return(
      0.90 -
      0.20 *
      (soc - 0.70) /
      0.10
    )
  }


  if (soc < 0.90) {

    return(
      0.70 -
      0.35 *
      (soc - 0.80) /
      0.10
    )
  }


  if (soc < 0.98) {

    return(
      0.35 -
      0.30 *
      (soc - 0.90) /
      0.08
    )
  }


  0.05
}


# ------------------------------------------------------------
# Battery current limit
# ------------------------------------------------------------

current_power_limit <- function(
    config,
    state
) {

  voltage <-
    battery_voltage(
      config,
      state
    )

  power_kw <-
    voltage *
    config$maximum_current /
    1000

  min(
    power_kw,
    config$maximum_charge_power_kw
  )
}


# ------------------------------------------------------------
# Thermal charging limit
# ------------------------------------------------------------

thermal_power_limit <- function(
    config,
    state
) {

  factor <-
    temperature_factor(
      config,
      state$temperature
    )

  config$maximum_charge_power_kw *
    factor
}


# ------------------------------------------------------------
# SOC power limit
# ------------------------------------------------------------

soc_power_limit <- function(
    config,
    state
) {

  factor <-
    soc_power_factor(
      state$soc
    )

  config$maximum_charge_power_kw *
    factor
}


# ------------------------------------------------------------
# Overall charging limit
# ------------------------------------------------------------

maximum_charge_power <- function(
    config,
    state,
    charger
) {

  charger_limit <-
    charger$maximum_power_kw


  current_limit <-
    current_power_limit(
      config,
      state
    )


  thermal_limit <-
    thermal_power_limit(
      config,
      state
    )


  soc_limit <-
    soc_power_limit(
      config,
      state
    )


  min(
    charger_limit,
    current_limit,
    thermal_limit,
    soc_limit
  )
}


# ------------------------------------------------------------
# Battery heat generation
#
# Simplified I²R model.
# ------------------------------------------------------------

battery_heat <- function(
    state,
    power_kw
) {

  voltage <-
    state$voltage

  current <-
    power_kw * 1000 /
    voltage

  # Approximate internal resistance.
  resistance <- 0.012

  heat_kw <-
    current^2 *
    resistance /
    1000

  heat_kw
}


# ------------------------------------------------------------
# Cooling model
# ------------------------------------------------------------

cooling_power <- function(
    config,
    state
) {

  temperature_error <-
    max(
      0,
      state$temperature -
        config$optimal_temperature
    )


  cooling <-
    temperature_error /
    config$thermal_resistance


  clamp(
    cooling,
    0,
    config$cooling_power_kw
  )
}


# ------------------------------------------------------------
# Thermal state update
# ------------------------------------------------------------

update_temperature <- function(
    config,
    state,
    power_kw
) {

  heat <-
    battery_heat(
      state,
      power_kw
    )


  cooling <-
    cooling_power(
      config,
      state
    )


  net_heat_kw <-
    heat - cooling


  # kW / kWh-per-degree gives
  # approximate °C/sec behaviour.
  temperature_change <-
    net_heat_kw /
    config$thermal_mass *
    config$dt


  state$temperature <-
    state$temperature +
    temperature_change


  state$temperature <-
    max(
      state$temperature,
      -20
    )


  state
}


# ------------------------------------------------------------
# SOC update
# ------------------------------------------------------------

update_soc <- function(
    config,
    state,
    power_kw
) {

  energy <-
    power_kw *
    config$dt /
    3600


  stored_energy <-
    energy *
    config$charge_efficiency


  state$soc <-
    state$soc +
    stored_energy /
    config$capacity_kwh


  state$soc <-
    clamp(
      state$soc,
      0,
      config$maximum_soc
    )


  state$energy_added_kwh <-
    state$energy_added_kwh +
    stored_energy


  state
}


# ------------------------------------------------------------
# Fast-charge controller
# ------------------------------------------------------------

fast_charge_controller <- function(
    config,
    state,
    charger
) {

  if (
    state$soc >=
    config$target_soc
  ) {

    return(0)
  }


  maximum_charge_power(
    config,
    state,
    charger
  )
}


# ------------------------------------------------------------
# Charging update
# ------------------------------------------------------------

charging_step <- function(
    config,
    state,
    charger
) {

  power <-
    fast_charge_controller(
      config,
      state,
      charger
    )


  state$voltage <-
    battery_voltage(
      config,
      state
    )


  state$current <-
    power *
    1000 /
    state$voltage


  state$power_kw <-
    power


  state$charging <-
    power > 0


  state <-
    update_temperature(
      config,
      state,
      power
    )


  state <-
    update_soc(
      config,
      state,
      power
    )


  state
}


# ------------------------------------------------------------
# RUN SIMULATION
# ------------------------------------------------------------

state <- battery_state

history <- data.frame()

time_seconds <- 0


while (
  state$soc <
  battery_config$target_soc
) {

  state <-
    charging_step(
      battery_config,
      state,
      charger
    )


  history <- rbind(
    history,
    data.frame(
      time_s = time_seconds,
      soc = state$soc,
      temperature = state$temperature,
      voltage = state$voltage,
      current = state$current,
      power_kw = state$power_kw,
      energy_kwh = state$energy_added_kwh
    )
  )


  time_seconds <-
    time_seconds +
    battery_config$dt


  # Safety timeout
  if (time_seconds > 3600) {
    break
  }
}


# ------------------------------------------------------------
# RESULTS
# ------------------------------------------------------------

cat(
  "Fast-charge completed\n"
)

cat(
  "Time:",
  round(time_seconds / 60, 1),
  "minutes\n"
)

cat(
  "Final SOC:",
  round(state$soc * 100, 1),
  "%\n"
)

cat(
  "Final temperature:",
  round(state$temperature, 1),
  "C\n"
)

cat(
  "Energy added:",
  round(state$energy_added_kwh, 2),
  "kWh\n"
)

cat(
  "Peak power:",
  round(max(history$power_kw), 1),
  "kW\n"
)

cat(
  "Peak current:",
  round(max(history$current), 1),
  "A\n"
)
