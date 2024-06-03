% scale(1000) import("thumb_3dprint.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
translate([16.5,62,0])
rotate([90,0,100])
cube([42, 25, 2], center=true);
translate([17,21,0])
rotate([90,0,80])
cube([42, 25, 2], center=true);
// sphere(10);
