//////////////////////////////////////////////////////////////
///
/// This work is licensed under the Creative Commons 
/// Attribution-NonCommercial-ShareAlike 4.0 International
/// License. To view a copy of this license,
/// visit https://creativecommons.org/licenses/by-nc-sa/4.0/.
///
/// Copyright 2020 erwin.rieger@ibrieger.de
///
//////////////////////////////////////////////////////////////
///
//D Holder for two Ultimaker UM2 feeders arranged in series
//D for increased grip and better control of the filament. 
//D Poor man's bondtech ;-)
///

// Smoother circles
$fa=0.1;
$fs=0.5;

fansize = 50+5;
fansizeHalbe = fansize/2;

innerFansize = 49;

motorHoleSpacing = 31;
motorHoleSpacingHalbe = motorHoleSpacing/2;

// Wandstärke
ws = 5;

// Feeder ist nicht symmetrisch, oberer rand:
feederRandOben = 7;

hAchse = fansize - feederRandOben - motorHoleSpacingHalbe;

dmounthole = 3.5;

module motorPlate() {

    motorHole = 23;

    motorHoleDiameter = 3.5;

    difference(){
        cube([fansize, fansize, ws]);

        translate([hAchse, fansizeHalbe, -1])
            cylinder(r=motorHole/2, h=ws+2);
    
        // motor mount holes
        translate([hAchse-motorHoleSpacing/2, fansizeHalbe-motorHoleSpacing/2,-1]) cylinder(d=motorHoleDiameter, h=ws+2);
        translate([hAchse-motorHoleSpacing/2, fansizeHalbe+motorHoleSpacing/2,-1]) cylinder(d=motorHoleDiameter, h=ws+2);
        translate([hAchse+motorHoleSpacing/2, fansizeHalbe-motorHoleSpacing/2,-1]) cylinder(d=motorHoleDiameter, h=ws+2);
        translate([hAchse+motorHoleSpacing/2, fansizeHalbe+motorHoleSpacing/2,-1]) cylinder(d=motorHoleDiameter, h=ws+2);
    }
}

module dreikant() {

    linear_extrude(height = 20, center = false, convexity = 10)
        polygon( points=[
            [-0.001, -ws/2-0.001],
            [ws+3.5, -ws/2-0.001],
            [ws+3.5, ws/2],
        ]);
}

module motorPlateKanal() {

  translate([5,0,0])
     rotate([0,-90,0])
        difference () {
          motorPlate();
          // Luftkanal
          translate([hAchse-20/2, fansizeHalbe, 0])
              cube([20, fansizeHalbe, ws]);
        }
}

module fanMount() {

    m_hole_dist = 40;
    fan_m_hole_dia = 3;

    difference() {
        rotate([90,0,0]) 
            union() {
                difference() {
                    cube([fansize, fansize, ws], center=true);
                    cylinder(d=innerFansize, h=ws, center=true);
                    translate([m_hole_dist/2, m_hole_dist/2, 0]) cylinder(d=fan_m_hole_dia, h=ws, center=true);
                    translate([-m_hole_dist/2, m_hole_dist/2, 0]) cylinder(d=fan_m_hole_dia, h=ws, center=true);
                    translate([m_hole_dist/2, -m_hole_dist/2, 0]) cylinder(d=fan_m_hole_dia, h=ws, center=true);
                    translate([-m_hole_dist/2, -m_hole_dist/2, 0]) cylinder(d=fan_m_hole_dia, h=ws, center=true);
                }
                translate([-fansizeHalbe+ws/2, 0])
                    cube([ws, fansize, ws], center=true);
            }
        translate([-fansizeHalbe, 0, hAchse-20/2-fansizeHalbe])
            dreikant();
    }
}

module half() {

    motorPlateKanal();

    difference() {
        cube([fansize, fansize, ws]);

        translate([fansizeHalbe-motorHoleSpacingHalbe, fansizeHalbe-motorHoleSpacingHalbe, 0]) cylinder(d=dmounthole, h=ws);
        translate([fansizeHalbe+motorHoleSpacingHalbe, fansizeHalbe-motorHoleSpacingHalbe, 0]) cylinder(d=dmounthole, h=ws);
        translate([fansizeHalbe-motorHoleSpacingHalbe, fansizeHalbe+motorHoleSpacingHalbe, 0]) cylinder(d=dmounthole, h=ws);
        translate([fansizeHalbe+motorHoleSpacingHalbe, fansizeHalbe+motorHoleSpacingHalbe, 0]) cylinder(d=dmounthole, h=ws);
    }

    translate([0, fansize, 0])
      difference() {
        translate([fansize/2, -ws/2, fansize/2])
            fanMount();
    }

    // Test-motor, debug
    motorWidth = 42;
    if ($preview) {
        translate([ws+75/2, fansizeHalbe, hAchse])
            #cube([75, motorWidth, motorWidth], center=true);
        }
}

half();
mirror([0, 1, 0]) half();




