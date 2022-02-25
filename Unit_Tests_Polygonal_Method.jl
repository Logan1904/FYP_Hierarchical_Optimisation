using Test
using Random
import Polygonal_Method
import Functions

@testset "Area Function" begin
    # (1) Non-intersecting circles
    # Circle A centred at (-1,-1) with Radius = 1
    # Circle B centred at (1,1) with Radius = 1
    z = [-1,1,-1,1,1,1]

    @test Polygonal_Method.Area(z) == 2*(pi*1*1)

    # (2) Tangent circles
    # Circle A centred at (-1,0) with Radius = 1
    # Circle B centred at (1,0) with Radius = 1
    z = [-1,1,0,0,1,1]

    @test Polygonal_Method.Area(z) == 2*(pi*1*1)

    # (3) Intersecting circles
    # Circle A centred at (0,0) with Radius = 1
    # Circle B centred at (1,0) with Radius = 1
    z = [0,1,0,0,1,1]

    @test isapprox(Polygonal_Method.Area(z), (4*pi/3) + (sqrt(3)/2))

    # (4) Coincident circles
    x, y, R = rand(3)
    z = [x,x,y,y,R,R]

    @test isapprox(Polygonal_Method.Area(z), pi*R*R)

end;