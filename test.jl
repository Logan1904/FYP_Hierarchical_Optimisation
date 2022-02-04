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

    #@test Functions.contained(A,A) == nothing

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
    x,y = rand(-1000:1000,2) #random points on a circle
    R = rand(1:1000,2)[1]
    N = rand(2:1000,1)[1]
    angles = LinRange(0,2*pi,N)

    True_Points = [Functions.Point(x + R*cos(angle), y + R*sin(angle),[],false) for angle in angles]
    Shuffled_Points = shuffle!(True_Points)
    
    Output_Points = Functions.sort_acw(Shuffled_Points,x,y)

    while Output_Points[1] != True_Points[1]
        Output_Points = circshift(Output_Points,1)
    end

    @test Output_Points == True_Points


    line = LinRange(x,y,N) #straight line at y=1
    y = 1

    True_Points = [Functions.Point(thispoint,y,[],false) for thispoint in line]
    Shuffled_Points = shuffle!(True_Points)

    Output_Points = Functions.sort_acw(Shuffled_Points,0,0)

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
    angles = LinRange(0,2*pi,N)

    True_Points = [Functions.Point(x + R*cos(angle), y + R*sin(angle),[],false) for angle in angles]
    Shuffled_Points = shuffle!(True_Points)

    Output_Points = Functions.sort_asc_angle(A,Shuffled_Points)

    @test Output_Points == True_Points
end
