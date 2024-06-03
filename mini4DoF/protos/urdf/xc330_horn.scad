% scale(1000) import("xc330_horn.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
translate([0,1.5,0])
rotate([90,0,0])
cylinder(r=8, h=2, center=true);
// sphere(10);
