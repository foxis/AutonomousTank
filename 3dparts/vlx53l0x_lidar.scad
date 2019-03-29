sensor_w = 12.5;
sensor_h = 24.7;
num_sensors = 12;
$fn = 20;

mount_w = 1;
base_r = 30;
base_h = 3;
base_screw_r = 1.5;
sensors_r = sensor_w / (2 * sin(180 / num_sensors));
sensor_r = sensors_r * cos(180 / num_sensors);
outer_r = sensors_r;
inner_r = sensor_r - mount_w;

module sensor_cutout() {
    for (i = [0:num_sensors])
        rotate([0, 0, i*360/num_sensors]) translate([sensor_r, -sensor_w/2, base_h]) cube([10, sensor_w, sensor_h+1]);
}

module base() {
    cylinder(r=base_r, h=base_h, $fn=40);
    difference() {
        cylinder(r=outer_r, h=base_h + sensor_h, $fn=100);
        translate([0,0,base_h]) cylinder(r=inner_r-1, h=sensor_h+1, $fn=40);
    }
}

module holes_cutout() {
    cylinder(r=base_screw_r, h=base_h+.1);
    d = (base_r + outer_r) / 2;
    translate([d, 0, 0]) cylinder(r=base_screw_r, h=base_h+.1);
    translate([-d, 0, 0]) cylinder(r=base_screw_r, h=base_h+.1);
    translate([0, d, 0]) cylinder(r=base_screw_r, h=base_h+.1);
    translate([0, -d, 0]) cylinder(r=base_screw_r, h=base_h+.1);
}

difference() {
    base();
    sensor_cutout();
    holes_cutout();
}
