# eusbullet

EusLisp Interface for the Bullet Physics.

## Build from source

Just `catkin build` this package. You do not need to install bullet manually, and you do not need to remove it even if already exists.


## Collisoin distance calculation

#### How to use

```
roseus
(load "package://eusbullet/euslisp/eusbullet.l")
(bt-collision-distance obj1 obj2) ;; obj1, obj2 can be body or bodyset-link
```

#### Sample

```
roscd eusbullet/euslisp/sample
roseus sample-collision-distance.l
(sample-collision-distance-body)
(sample-collision-distance-link)
(sample-collision-distance-2d-analytical)
```
