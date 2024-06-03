% scale(1000) import("fr12-h101k.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
translate([-26,0,0])
cube([4, 41, 24], center=true);
translate([-13,-19,0])
cube([30, 3, 24], center=true);
translate([-13,19,0])
cube([30, 3, 24], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);
