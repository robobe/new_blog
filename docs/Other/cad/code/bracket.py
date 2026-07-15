import cadquery as cq


length = 80
width = 30
thickness = 8
hole_diameter = 8

part = (
    cq.Workplane("XY")
    .box(length, width, thickness)
    .edges("|Z")
    .fillet(3)
    .faces(">Z")
    .workplane()
    .pushPoints([(-25, 0), (25, 0)])
    .hole(hole_diameter)
)

show_object(part)
