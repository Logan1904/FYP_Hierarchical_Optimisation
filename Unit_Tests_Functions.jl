using Test
using Random
using LinearAlgebra
import Functions

@testset "Euclidean Distance Function" begin
    x1,y1,x2,y2 = rand(-1000:1000,4)
    R1,R2 = rand(1:1000,2)

    A = Functions.Circle(x1,y1,R1,[],[],false)
    B = Functions.Circle(x2,y2,R2,[],[],false)

    @test Functions.distance(A,B) == sqrt((A.x-B.x)^2 + (A.y-B.y)^2)
end

@testset "Coincident Function" begin
    x, y, R = rand(3)
    A = Functions.Circle(x,y,R,[],[],false)
    B = Functions.Circle(x,y,R,[],[],false)
    C = Functions.Circle(x+0.1,y+0.1,R+0.1,[],[],false)

    @test Functions.coincident(A,B) == true
    @test Functions.coincident(A,C) == false
end

@testset "Intersection Function" begin
    A = Functions.Circle(0,0,1,[],[],false)
    B = Functions.Circle(1,0,1,[],[],false)
    C = Functions.Circle(2,0,1,[],[],false)
    D = Functions.Circle(3,0,1,[],[],false)

    @test Functions.intersection(A,A) === nothing
    @test Functions.intersection(A,B) == (0.5,-0.5*sqrt(3),0.5,+0.5*sqrt(3))
    @test Functions.intersection(A,C) === nothing
    @test Functions.intersection(A,D) === nothing
end

@testset "Contained Function" begin
    A = Functions.Circle(0,0,1,[],[],false)
    B = Functions.Circle(0,0,0.5,[],[],false)
    C = Functions.Circle(1,0,1,[],[],false)
    D = Functions.Circle(2,0,1,[],[],false)
    E = Functions.Circle(0.5,0,0.5,[],[],false)

    Functions.contained(A,B)
    @test A.Contained == false
    @test B.Contained == true

    Functions.contained(A,C)
    @test A.Contained == false
    @test C.Contained == false

    Functions.contained(A,D)
    @test A.Contained == false
    @test D.Contained == false

    Functions.contained(A,E)
    @test A.Contained == false
    @test E.Contained == true
end

@testset "Boundary Function" begin
    A = Functions.Circle(0,0,1,[],[],false)
    Point1 = Functions.Point(0,1,[],false)
    Point2 = Functions.Point(0,0,[],false)
    Point3 = Functions.Point(1/sqrt(2),1/sqrt(2),[],false)

    @test Functions.boundary(A,Point1) == true
    @test Functions.boundary(A,Point2) == false
    @test Functions.boundary(A,Point3) == true
end

@testset "ACW Sort Function" begin
    x = Float64(rand(-1000:1000,1)[1]) #random points on a circle
    y = Float64(rand(-1000:1000,1)[1])
    R = rand(1:1000,2)[1]
    N = rand(2:1000,1)[1]
    angles = LinRange(0.0,2*pi,N)

    True_Points = [Functions.Point(x + R*cos(angle), y + R*sin(angle),[],false) for angle in angles]

    Output_Points = shuffle(True_Points)
    Functions.sort_acw!(Output_Points,x,y)

    while Output_Points[1] != True_Points[1]
        Output_Points = circshift(Output_Points,1)
    end

    @test Output_Points == True_Points
end

@testset "ACW Ascending Sort Function" begin
    x,y = rand(-1000:1000,2)
    R = rand(1:1000,2)[1]

    A = Functions.Circle(x,y,R,[],[],false)

    N = rand(2:1000,1)[1]
    angles = LinRange(0.,2*pi,N)

    True_Points = [Functions.Point(x + R*cos(angle), y + R*sin(angle),[],false) for angle in angles]

    Output_Points = shuffle(True_Points)
    Functions.sort_asc_angle!(A,Output_Points)

    @test Output_Points == True_Points
end;

@testset "Shoelace Function" begin
    square = [Functions.Point(-1,-1,[],false),
              Functions.Point(-1,1,[],false),
              Functions.Point(1,1,[],false),
              Functions.Point(1,-1,[],false)]
    
    x_circle, y_circle = Functions.draw(Functions.Circle(0,0,2,[],[],false), 0.0, 2*pi)
    circle = [Functions.Point(x_circle[i],y_circle[i],[],false) for i in 1:length(x_circle)]

    @test Functions.shoelace(square) == 4
    @test isapprox(Functions.shoelace(circle), pi*2*2, atol=0.1)
end;