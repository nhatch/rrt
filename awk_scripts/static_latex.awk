END {
  for (o in orders) {
    order = orders[o]
    for (m in modes) {
      mode = modes[m]
      for (obs in dyn_obstacles) {
        obstacles = dyn_obstacles[obs]
        for (t in tasks) {
          task = tasks[t]
          for (r in robots) {
            robot = robots[r]
            key = robot SUBSEP mode
            successes[key] += results_succeeded[mode,task,robot,order,obstacles]
            fails[key] += results_failed[mode,task,robot,order,obstacles]
            colfrees[key] += results_no_collision[mode,task,robot,order,obstacles]
            norm_denom[key] += results_normalized_cost_total[mode,task,robot,order,obstacles]
            norm_cost[key] += results_normalized_cost[mode,task,robot,order,obstacles]
            norm_cost_var[key] += results_normalized_cost_var[mode,task,robot,order,obstacles]
          }
        }
      }
    }
  }
  human_readable = 0
  if (human_readable == 1) {
    for (r in robots) {
      robot = robots[r]
      print robot
      for (m in modes) {
        mode = modes[m]
        key = robot SUBSEP mode
        print "  " mode
        print "    Failure rate: " (1 - (successes[key] / (successes[key]+fails[key])))
        print "    Collision rate: " (1 - (colfrees[key] / successes[key]))
        print "    Normalized cost: " (norm_cost[key] / norm_denom[key]) " over total graphs: " norm_denom[key]
        print "    Normalized cost std: " sqrt(norm_cost_var[key] / norm_denom[key])
      }
    }
  }
  if (human_readable == 0) {
    rowhead_format["point"] = "Point"
    rowhead_format["stick"] = "Stick"
    mode_format["naive"] = "\\texttt{naive:}"
    mode_format["min"] = "\\\\ \\texttt{min:~~}"
    mode_format["full"] = "\\\\ \\texttt{full:~}"
    for (r in robots) {
      robot = robots[r]
      print "\\hline"
      print rowhead_format[robot] " &"
      printf "%s","\\makecell{"
      for (m in modes) {
        mode = modes[m]
        key = robot SUBSEP mode
        # Failure rate
        printf "%s $%.1f$", mode_format[mode], 100*(1 - (successes[key] / (successes[key]+fails[key])))
      }
      print "}"
      print "&"
      printf "%s","\\makecell{"
      for (m in modes) {
        mode = modes[m]
        key = robot SUBSEP mode
        # Collision rate
        printf "%s $%.1f$", mode_format[mode], 100*(1 - (colfrees[key] / successes[key]))
      }
      print "}"
      print "&"
      printf "%s","\\makecell{"
      for (m in modes) {
        mode = modes[m]
        key = robot SUBSEP mode
        # Normalized cost
        mean = (norm_cost[key] / norm_denom[key])
        var = sqrt(norm_cost_var[key] / norm_denom[key])
        if (mode != "min") printf "%s $%.3f\\pm%.3f$", mode_format[mode], mean, var
        if (mode == "min") printf "%s ---~~~~~~~~~~~~~~~", mode_format[mode]
      }
      print "}"
      print "\\\\"
    }
  }
}
