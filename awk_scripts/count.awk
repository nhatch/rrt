BEGIN {
  FS = " "
}
/MEAN4VARIANCE/ {
  means[$2,$3,$4,$5,$6] = $7
}
/TASK NAME|Found feasible path/ {
  nc_min = seed_results_no_collision["min"]
  nc_full = seed_results_no_collision["full"]
  nc_naive = seed_results_no_collision["naive"]
  cost_min = seed_results_path_cost["min"]
  cost_full = seed_results_path_cost["full"]
  cost_naive = seed_results_path_cost["naive"]
  if (nc_min >= 3) {
    results_normalized_cost_total["min",task,robot,order,obstacles] += 1
    results_normalized_cost["min",task,robot,order,obstacles] += 1.0
    results_normalized_cost_var["min",task,robot,order,obstacles] += 0.0

    if (nc_full >= 3) {
      normalized_cost_full = (cost_full / nc_full) / (cost_min / nc_min)
      mean = means["full",task,robot,order,obstacles]
      variance = (normalized_cost_full - mean) * (normalized_cost_full - mean)
      results_normalized_cost_total["full",task,robot,order,obstacles] += 1
      results_normalized_cost["full",task,robot,order,obstacles] += normalized_cost_full
      results_normalized_cost_var["full",task,robot,order,obstacles] += variance
    }
    if (nc_naive >= 3) {
      normalized_cost_naive = (cost_naive / nc_naive) / (cost_min / nc_min)
      mean = means["naive",task,robot,order,obstacles]
      variance = (normalized_cost_naive - mean) * (normalized_cost_naive - mean)
      results_normalized_cost_total["naive",task,robot,order,obstacles] += 1
      results_normalized_cost["naive",task,robot,order,obstacles] += normalized_cost_naive
      results_normalized_cost_var["naive",task,robot,order,obstacles] += variance
    }
  }
  for (m in modes) {
    seed_results_no_collision[modes[m]] = 0
    seed_results_path_cost[modes[m]] = 0
  }
}
/NAIVE/ { mode = "naive" }
/MPPI \(full\)/ { mode = "full" }
/MPPI \(min\)/ { mode = "min" }
/TASK NAME: gate/ { task = "gate" }
/TASK NAME: bugtrap/ { task = "bugtrap" }
/TASK NAME: blob/ { task = "blob" }
/TASK NAME: forest/ { task = "forest" }
/TASK MODEL IS POINT ROBOT: 1/ { robot = "point" }
/TASK MODEL IS POINT ROBOT: 0/ { robot = "stick" }
/TASK MODEL IS SECOND ORDER: 1/ { order = "second" }
/TASK MODEL IS SECOND ORDER: 0/ { order = "first" }
/TASK INCLUDES DYNAMIC OBSTACLES: 1/ { obstacles = "dynamic" }
/TASK INCLUDES DYNAMIC OBSTACLES: 0/ { obstacles = "static" }
/Success: *0/ {
  last_collided = 1
  last_success = 0
  results_failed[mode,task,robot,order,obstacles] += 1
}
/Success: *1/ {
  last_success = 1
  last_collided = 1
  results_succeeded[mode,task,robot,order,obstacles] += 1
}
/\(0 collisions\)/ {
  last_collided = 0
  if (last_success == 1) {
    results_no_collision[mode,task,robot,order,obstacles] += 1
    seed_results_no_collision[mode] += 1
  }
}
/Total cost:/ {
  if (last_success == 1 && last_collided == 0) {
    results_path_cost[mode,task,robot,order,obstacles] += $3
    seed_results_path_cost[mode] += $3
  }
}
/Control steps:/ {
  if (last_success == 1 && last_collided == 0) {
    results_control_steps[mode,task,robot,order,obstacles] += $3
  }
}
/Min graph steps:/ {
  if (last_success == 1 && last_collided == 0) {
    results_shortsteps[mode,task,robot,order,obstacles] += $4
  }
}
