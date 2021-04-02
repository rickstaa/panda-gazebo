# Geometrical calculations

A guide on how to calculate geometrical properties like moment of inertial, center of gravity and mass can be found [here](http://gazebosim.org/tutorials?tut=inertia&cat=build_robot). For our robot, this was already done by [@mkrizmancic](https://github.com/mkrizmancic/franka_gazebo). Here a small validation.

## Robot specs

A datasheet for the robot is published [here](https://s3-eu-central-1.amazonaws.com/franka-de-uploads-staging/uploads/2018/05/2018-05-datasheet-panda.pdf). It states that the total mass of the ar is ~18 kg while the mass of the hand is ~0.7 kg.

### Arm

#### Volumes

-   j0: `2.435092 m^3`
-   j1: `2.293251 m^3`
-   j2: `2.312302 m^3`
-   j3: `2.020883 m^3`
-   j4: `2.005719 m^3`
-   j5: `2.275315 m^3`
-   j6: `1.263041 m^3`
-   j7: `0.422458 m^3`

Total: `15.028061000000001 m^3`

#### Masses

-   m1: `(2.435092/15.028061000000001)*18=2.9166541179198036 kg`
-   m2: `(2.293251/15.028061000000001)*18=2.746762739384675 kg`
-   m3: `(2.312302/15.028061000000001)*18=2.7695812520324474 kg`
-   m4: `(2.020883/15.028061000000001)*18=2.4205314311673343 kg`
-   m5: `(2.005719/15.028061000000001)*18=2.402368608964257 kg`
-   m6: `(2.275315/15.028061000000001)*18=2.7252797283694816 kg`
-   m7: `(1.263041/15.028061000000001)*18=1.5128191188470688 kg`
-   m8: `(0.422458/15.028061000000001)*18=0.506003003314932 kg`

#### Inertias, center of gravities

Fill in in the inertial_xml_marker.py script to account for scaling.

### Hand

#### Volumes

-   Hand: `0.000488 m^3`
-   Finger1/2: `0.000011 m^3`

Total: `0.000510 m^2`

#### Masses

-   Hand: `(0.000488/0.000510)*0.7=0.6698039215686273 kg`
-   Finger1/2: `(0.000011/0.000510)*0.7=0.015098039215686272 kg`

#### Inertias, center of gravities

Fill in in the inertial_xml_marker.py script to account for scaling.
