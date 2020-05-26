//page size
w=297;
h=420;

// battery "single" size ( in this case it's a pair ) 
batt_w = 18.5;
batt_h = 70;

// frame outer dimension
perimeter = 11;

// inner-spacing
wide_inner = 10;
thin_inner = 10;

// batteries in a stack = 20
battstack = 1;

// holeoffsets
holeoffset1 = 6;
holeoffset2 = 16;


//echo((h-(batt_h*battstack))/2);

difference(){ 
square([w,h],centre=false); // A3 sheet to start on.

// three rows out from centre.
for (i = [-140,-20,100]){ 

translate ( [0,i,0] ) {     
offset1 = perimeter;
translate([offset1,(h-(batt_h*battstack))/2,0]) square([batt_w,batt_h*battstack],centre=false);
translate([offset1+6,(h-(batt_h*battstack))/2-3,0]) square([6,1],centre=false);
translate([offset1+6,(h-(batt_h*battstack))/2-5,0]) square([6,1],centre=false);
translate([offset1+6,(h-(batt_h*battstack))/2+batt_h+3,0]) square([6,1],centre=false);
translate([offset1+6,(h-(batt_h*battstack))/2+batt_h+5,0]) square([6,1],centre=false);

offset2 = offset1+batt_w+wide_inner;
translate([offset2,(h-(batt_h*battstack))/2,0]) square([batt_w,batt_h*battstack],centre=false);
translate([offset2+6,(h-(batt_h*battstack))/2-3,0]) square([6,1],centre=false);
translate([offset2+6,(h-(batt_h*battstack))/2-5,0]) square([6,1],centre=false);
translate([offset2+6,(h-(batt_h*battstack))/2+batt_h+3,0]) square([6,1],centre=false);
translate([offset2+6,(h-(batt_h*battstack))/2+batt_h+5,0]) square([6,1],centre=false);
    
offset3 = offset2+batt_w+thin_inner;
translate([offset3,(h-(batt_h*battstack))/2,0]) square([batt_w,batt_h*battstack],centre=false);
translate([offset3+6,(h-(batt_h*battstack))/2-3,0]) square([6,1],centre=false);
translate([offset3+6,(h-(batt_h*battstack))/2-5,0]) square([6,1],centre=false);
translate([offset3+6,(h-(batt_h*battstack))/2+batt_h+3,0]) square([6,1],centre=false);
translate([offset3+6,(h-(batt_h*battstack))/2+batt_h+5,0]) square([6,1],centre=false);
    
offset4 = offset3+batt_w+wide_inner;
translate([offset4,(h-(batt_h*battstack))/2,0]) square([batt_w,batt_h*battstack],centre=false);
translate([offset4+6,(h-(batt_h*battstack))/2-3,0]) square([6,1],centre=false);
translate([offset4+6,(h-(batt_h*battstack))/2-5,0]) square([6,1],centre=false);
translate([offset4+6,(h-(batt_h*battstack))/2+batt_h+3,0]) square([6,1],centre=false);
translate([offset4+6,(h-(batt_h*battstack))/2+batt_h+5,0]) square([6,1],centre=false);
    
offset5 = offset4+batt_w+thin_inner;
translate([offset5,(h-(batt_h*battstack))/2,0]) square([batt_w,batt_h*battstack],centre=false);
translate([offset5+6,(h-(batt_h*battstack))/2-3,0]) square([6,1],centre=false);
translate([offset5+6,(h-(batt_h*battstack))/2-5,0]) square([6,1],centre=false);
translate([offset5+6,(h-(batt_h*battstack))/2+batt_h+3,0]) square([6,1],centre=false);
translate([offset5+6,(h-(batt_h*battstack))/2+batt_h+5,0]) square([6,1],centre=false);

offset6 = offset5+batt_w+wide_inner;
translate([offset6,(h-(batt_h*battstack))/2,0]) square([batt_w,batt_h*battstack],centre=false);
translate([offset6+6,(h-(batt_h*battstack))/2-3,0]) square([6,1],centre=false);
translate([offset6+6,(h-(batt_h*battstack))/2-5,0]) square([6,1],centre=false);
translate([offset6+6,(h-(batt_h*battstack))/2+batt_h+3,0]) square([6,1],centre=false);
translate([offset6+6,(h-(batt_h*battstack))/2+batt_h+5,0]) square([6,1],centre=false);

offset7 = offset6+batt_w+thin_inner;
translate([offset7,(h-(batt_h*battstack))/2,0]) square([batt_w,batt_h*battstack],centre=false);
translate([offset7+6,(h-(batt_h*battstack))/2-3,0]) square([6,1],centre=false);
translate([offset7+6,(h-(batt_h*battstack))/2-5,0]) square([6,1],centre=false);
translate([offset7+6,(h-(batt_h*battstack))/2+batt_h+3,0]) square([6,1],centre=false);
translate([offset7+6,(h-(batt_h*battstack))/2+batt_h+5,0]) square([6,1],centre=false);

offset8 = offset7+batt_w+wide_inner;
translate([offset8,(h-(batt_h*battstack))/2,0]) square([batt_w,batt_h*battstack],centre=false);
translate([offset8+6,(h-(batt_h*battstack))/2-3,0]) square([6,1],centre=false);
translate([offset8+6,(h-(batt_h*battstack))/2-5,0]) square([6,1],centre=false);
translate([offset8+6,(h-(batt_h*battstack))/2+batt_h+3,0]) square([6,1],centre=false);
translate([offset8+6,(h-(batt_h*battstack))/2+batt_h+5,0]) square([6,1],centre=false);

offset9 = offset8+batt_w+thin_inner;
translate([offset9,(h-(batt_h*battstack))/2,0]) square([batt_w,batt_h*battstack],centre=false);
translate([offset9+6,(h-(batt_h*battstack))/2-3,0]) square([6,1],centre=false);
translate([offset9+6,(h-(batt_h*battstack))/2-5,0]) square([6,1],centre=false);
translate([offset9+6,(h-(batt_h*battstack))/2+batt_h+3,0]) square([6,1],centre=false);
translate([offset9+6,(h-(batt_h*battstack))/2+batt_h+5,0]) square([6,1],centre=false);

offset10 = offset9+batt_w+thin_inner;
translate([offset10,(h-(batt_h*battstack))/2,0]) square([batt_w,batt_h*battstack],centre=false);
translate([offset10+6,(h-(batt_h*battstack))/2-3,0]) square([6,1],centre=false);
translate([offset10+6,(h-(batt_h*battstack))/2-5,0]) square([6,1],centre=false);
translate([offset10+6,(h-(batt_h*battstack))/2+batt_h+3,0]) square([6,1],centre=false);
translate([offset10+6,(h-(batt_h*battstack))/2+batt_h+5,0]) square([6,1],centre=false);

}

}



//bus holes
//translate([offset2-wide_inner/2,holeoffset1,0]) circle(d=6.5);
//translate([offset2-wide_inner/2,h-holeoffset1,0]) circle(d=6.5);
//translate([offset4-wide_inner/2,holeoffset1,0]) circle(d=6.5);
//translate([offset4-wide_inner/2,h-holeoffset1,0]) circle(d=6.5);
//translate([offset6-wide_inner/2,holeoffset1,0]) circle(d=6.5);
//translate([offset6-wide_inner/2,h-holeoffset1,0]) circle(d=6.5);

// mount holes
translate([w*1/32,holeoffset2,0]) circle(d=6.5);
translate([w*11/32,holeoffset2,0]) circle(d=6.5);
translate([w*21/32,holeoffset2,0]) circle(d=6.5);
translate([w*31/32,holeoffset2,0]) circle(d=6.5);

translate([w*1/32,h-holeoffset2,0]) circle(d=6.5);
translate([w*11/32,h-holeoffset2,0]) circle(d=6.5);
translate([w*21/32,h-holeoffset2,0]) circle(d=6.5);
translate([w*31/32,h-holeoffset2,0]) circle(d=6.5);

translate([w*1/32,140,0]) circle(d=6.5);
translate([w*11/32,140,0]) circle(d=6.5);
translate([w*21/32,140,0]) circle(d=6.5);
translate([w*31/32,140,0]) circle(d=6.5);

translate([w*1/32,260,0]) circle(d=6.5);
translate([w*11/32,260,0]) circle(d=6.5);
translate([w*21/32,260,0]) circle(d=6.5);
translate([w*31/32,260,0]) circle(d=6.5);

//optional ruler markes on cutouts. 
    
} 