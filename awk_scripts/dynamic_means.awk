
END {
  # Means for dynamic results
  for (m in modes) {
    for (o in orders) {
      for (obs in dyn_obstacles) {
        norm_denom = 0
        norm_cost = 0
        for (r in robots) {
          for (t in tasks) {
            norm_denom += results_normalized_cost_total[modes[m],tasks[t],robots[r],orders[o],dyn_obstacles[obs]]
            norm_cost += results_normalized_cost[modes[m],tasks[t],robots[r],orders[o],dyn_obstacles[obs]]
          }
        }
        for (r in robots) {
          for (t in tasks) {
            print "MEAN4VARIANCE " modes[m] " " tasks[t] " " robots[r] " " orders[o] " " dyn_obstacles[obs] " " (norm_cost/norm_denom)
          }
        }
      }
    }
  }
}
