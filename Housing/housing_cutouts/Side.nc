%
(Side)
(T1  D=2 CR=0 - ZMIN=-3 - flat end mill)
G90 G94
G17
G21

(2D Contour1)
M9
T1 M6
S5000 M3
G54
M9
G0 X-63.88 Y-3.9
Z15
Z5
G1 Z2.712 F333.3
G19 G2 Y-4.1 Z2.512 J-0.2
G1 Y-4.3 F500
G17 G3 X-63.68 Y-4.5 I0.2
G1 X-60.5 Z1.355 F333.3
Y3.5 Z-1.557
X-64.432 Z-2.988
G18 G3 X-64.5 Z-3 I-0.068 K0.188
G1 X-69.5 F500
Y-4.5
X-60.5
Y3.5
X-64.5
G17 G3 X-64.7 Y3.3 J-0.2
G1 Y3.1
G19 G2 Y2.9 Z-2.8 K0.2
G0 Z15
G17
M9
M30
%
