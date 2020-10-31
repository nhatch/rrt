
END {
  # Means for static results
  order = "first"
  obstacles = "static"
  for (m in modes) {
    for (r in robots) {
      norm_denom = 0
      norm_cost = 0
      for (t in tasks) {
        norm_denom += results_normalized_cost_total[modes[m],tasks[t],robots[r],order,obstacles]
        norm_cost += results_normalized_cost[modes[m],tasks[t],robots[r],order,obstacles]
      }
      for (t in tasks) {
        print "MEAN4VARIANCE " modes[m] " " tasks[t] " " robots[r] " " order " " obstacles " " (norm_cost/norm_denom)
      }
    }
  }
}
