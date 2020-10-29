BEGIN { FS = " " }
/NAIVE/ { mode = "naive" }
/MPPI \(full\)/ { mode = "full" }
/MPPI \(min\)/ { mode = "min" }
/Total cost:/ {
  if (mode == "naive") naive_path += $3
  if (mode == "full") full_path += $3
  if (mode == "min") min_path += $3
}
/Control steps:/ {
  if (mode == "naive") naive_steps += $3
  if (mode == "full") full_steps += $3
  if (mode == "min") min_steps += $3
}
/Min graph steps:/ {
  if (mode == "naive") naive_shortsteps += $4
  if (mode == "full") full_shortsteps += $4
  if (mode == "min") min_shortsteps += $4
}
/\(0 collisions\)/ {
  if (mode == "naive") naive_colfree += 1
  if (mode == "full") full_colfree += 1
  if (mode == "min") min_colfree += 1
}
/Success: *1/ {
  if (mode == "naive") naive_success += 1
  if (mode == "full") full_success += 1
  if (mode == "min") min_success += 1
}
END {
  print "Note: order is (naive, MPPI with full graph, MPPI with shortest path)"
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
  print "Collision freedom:"
  print "  " naive_colfree
  print "  " full_colfree
  print "  " min_colfree
  print "Not stuck or lost:"
  print "  " naive_success
  print "  " full_success
  print "  " min_success
}
