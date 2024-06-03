% scale(1000) import("pinza_small.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
translate([0,40,6])
rotate([90,0,91])
cube([68, 10, 2], center=true);
translate([11,42,6])
rotate([90,0,110])
cube([68, 10, 2], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);
