BEGIN { FS = " " }
/NAIVE/ { mode = "naive" }
/MPPI \(full\)/ { mode = "full" }
/MPPI \(min\)/ { mode = "min" }
/Total cost:/ {
  if (last_success == 1) {
    if (mode == "naive") naive_path += $3
    if (mode == "full") full_path += $3
    if (mode == "min") min_path += $3
  }
}
/Control steps:/ {
  if (last_success == 1) {
    if (mode == "naive") naive_steps += $3
    if (mode == "full") full_steps += $3
    if (mode == "min") min_steps += $3
  }
}
/Min graph steps:/ {
  if (last_success == 1) {
    if (mode == "naive") naive_shortsteps += $4
    if (mode == "full") full_shortsteps += $4
    if (mode == "min") min_shortsteps += $4
  }
}
/\(0 collisions\)/ {
  if (last_success == 1) {
    if (mode == "naive") naive_colfree += 1
    if (mode == "full") full_colfree += 1
    if (mode == "min") min_colfree += 1
  }
}
/Success: *1/ {
  last_success = 1
  if (mode == "naive") naive_success += 1
  if (mode == "full") full_success += 1
  if (mode == "min") min_success += 1
}
/Success: *0/ {
  last_success = 0
  if (mode == "naive") naive_fail += 1
  if (mode == "full") full_fail += 1
  if (mode == "min") min_fail += 1
}
END {
  if (naive_success > 0) naive_path = naive_path / naive_success
  full_path = full_path / full_success
  min_path = min_path / min_success
  if (naive_success > 0) naive_steps = naive_steps / naive_success
  full_steps = full_steps / full_success
  min_steps = min_steps / min_success
  if (naive_success > 0) naive_shortsteps = naive_shortsteps / naive_success
  full_shortsteps = full_shortsteps / full_success
  min_shortsteps = min_shortsteps / min_success
  if (naive_success > 0) naive_colfree = naive_colfree / naive_success
  full_colfree = full_colfree / full_success
  min_colfree = min_colfree / min_success
  print "Note: order is:"
  print "  naive"
  print "  MPPI with full graph"
  print "  MPPI with shortest path"
  print "Got stuck or lost:"
  print "  " 0+naive_fail " of " naive_fail+naive_success
  print "  " full_fail " of " full_fail+full_success
  print "  " min_fail " of " min_fail+min_success
  print "Collided with an obstacle:"
  print "  " 1-naive_colfree
  print "  " 1-full_colfree
  print "  " 1-min_colfree
  print "Path lengths:"
  print "  " naive_path
  print "  " full_path
  print "  " min_path
  print "Control steps:"
  print "  " naive_steps
  print "  " full_steps
  print "  " min_steps
  print "Control steps on the shortest path:"
  print "  " naive_shortsteps
  print "  " full_shortsteps
  print "  " min_shortsteps
}
