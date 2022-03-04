module Base_Functions

using LinearAlgebra
using Random

"""
    struct Circle

A circle object

Attributes:
    - 'x::Float64':             x-position of the circle centre on a 2D Cartesian grid
    - 'y::Float64':             y-position of the circle centre on a 2D Cartesian grid
    - 'R::Float64':             Radius
    - 'Points::Vector{Int}':    Vector of intersection point indices
    - 'Circles::Vector{Int}':   Vector of intersection circles indices
    - 'Contained::Bool':        True if is contained by another circle

"""
mutable struct Circle
    x::Float64
    y::Float64
    R::Float64
    Points::Vector{Int}
    Circles::Vector{Int}
    Contained::Bool
end

"""
    struct Point

A point object

Attributes:
    - 'x::Float64':             x-position of the point on a 2D Cartesian grid
    - 'y::Float64':             y-position of the point on a 2D Cartesian grid
    - 'Circles::Vector{Int}':   Vector of circle indices that form point
    - 'ID::Bool':               True if point is on outer contour

"""
mutable struct Point
    x::Float64
    y::Float64
    Circles::Vector{Int}
    ID::Bool
end

"""
    make_circles(arr)

Returns a Vector of Circle objects

Arguments:
    - 'x{Vector{Float64}}': Vector of x-coordinates
    - 'y{Vector{Float64}}:  Vector of y-coordinates
    - 'R{Vector{Float64}}:  Vector of radius values
"""
function make_circles(x,y,R)
    N = Int(length(x))

    circles = Vector{Circle}()
    for i in 1:N
        push!(circles, Circle(x[i],y[i],R[i],[],[],false))
    end
    
    return circles
end

"""
    distance(A::Circle, B::Circle)

Returns the Euclidean distance between 2 Circle centres
"""
function distance(A::Circle,B::Circle)
    return sqrt((A.x-B.x)^2 + (A.y-B.y)^2)
end

"""
    coincident(A::Circle, B::Circle)

Returns 'true' if the 2 Circles are coincident, 'false' otherwise
"""
function coincident(A::Circle,B::Circle)
    d = round(distance(A,B), digits=1)
    if d == 0 && A.R == B.R
        return true
    else
        return false
    end
end

"""
    contained!(A::Circle, B::Circle)

Checks if a Circle is contained by another and modifies the attribute 'Circle.Contained' accordingly
"""
function contained!(A::Circle,B::Circle)
    d = distance(A,B)
    if d <= abs(A.R - B.R)
        if A.R < B.R
            A.Contained = true
        else
            B.Contained = true
        end
    end
end

"""
    intersection(A::Circle, B::Circle)

Returns the intersection coordinates of 2 Circles, in order 'x1,y1,x2,y2'
Returns 'nothing' if the 2 Circles do not intersect or are tangent
Calls 'contained!(A::Circle, B::Circle)' if one Circle is contained by the other Circle
"""
function intersection(A::Circle,B::Circle)
    d = distance(A,B)

    if d > A.R + B.R                # non-intersecting
        return nothing
    elseif d <= abs(A.R - B.R)      # one circle within another
        contained!(A,B)
        return nothing
    else
        a = (d^2+A.R^2-B.R^2)/(2*d)
        h = sqrt((A.R^2-a^2))

        varx = A.x + a*(B.x-A.x)/d
        vary = A.y + a*(B.y-A.y)/d

        x1 = varx + h*(B.y-A.y)/d
        y1 = vary - h*(B.x-A.x)/d
        x2 = varx - h*(B.y-A.y)/d
        y2 = vary + h*(B.x-A.x)/d

        if x1 == x2 && y1 == y2     # tangent circles -> we take it as non-intersecting
            return nothing
        end

        return x1,y1,x2,y2
    end
end

"""
    boundary(A::Circle, P::Point)

Returns 'true' if Point P is inside Circle A, 'false' otherwise
"""
function boundary(A::Circle,P::Point)
    x = P.x
    y = P.y
    
    if round((x-A.x)^2 + (y-A.y)^2 - A.R^2, digits=8) < 0
        return false
    else 
        return true
    end
end

"""
    sort_acw!(Array::Vector{Any}, mean_x::Float64, mean_y::Float64)

Sorts a Vector of Points and/or Circles in anticlockwise order
"""
function sort_acw!(Array::Vector{Any},mean_x::Float64,mean_y::Float64)
    N = Int(length(Array))

    for i in 1:N
        for j in i+1:N
            ax = Array[i].x
            ay = Array[i].y
            bx = Array[j].x
            by = Array[j].y

            det = (ax-mean_x)*(by-mean_y) - (bx-mean_x)*(ay-mean_y)

            if det < 0
                tmp = Points[i]
                Points[i] = Points[j]
                Points[j] = tmp
            end
        end
    end

    return Points
end

"""
    sort_asc_angle!(A::Circle, Array::Vector{Point})

Sorts a Vector of Points relative to a Circle in ascending order of angle
"""
function sort_asc_angle!(A::Circle, Array::Vector{Point})
    N = Int(length(Array))

    for i in 1:N
        for j in i+1:N
            point = Array[i]
            theta1 = mod(atan(Array[i].y-A.y,Array[i].x-A.x), 2*pi)
            theta2 = mod(atan(Array[j].y-A.y,Array[j].x-A.x), 2*pi)

            if theta2 < theta1
                tmp = Array[i]
                Array[i] = Array[j]
                Array[j] = tmp
            end
        end
    end

    return Array
end


# returns x and y vectors of a Circle object (for plotting)
function draw(A::Circle,theta1,theta2)
    if theta1 > theta2
        theta2 = theta2 + 2*pi
    end
    arr = LinRange(theta1,theta2,101)
    return A.x .+ A.R*cos.(arr), A.y .+ A.R*sin.(arr)
end



# sort a vector of Point objects relative to a Circle object in ascending order of Polar angle


# returns a Point object given a Circle object and Polar angle 
function point_on_circle(A::Circle,theta)
    x = A.x + A.R*cos(theta)
    y = A.y + A.R*sin(theta)

    return Functions.Point(x,y,[],0)
end

# get area from a sorted (ACW/CW) vector of points (Circle or Point objects) using Shoelace Method
function shoelace(Points)
    xarr = [point.x for point in Points]
    yarr = [point.y for point in Points]

    dum1 = dot(xarr,circshift(yarr,1))
    dum2 = dot(yarr,circshift(xarr,1))

    area = 0.5*broadcast(abs,(dum1-dum2))
    
    return area
end

# get area of a sector
# array is a vector of vectors, with each row [Circle, StartAngle, EndAngle]
function area_sector(array)
    circle,theta1,theta2 = array
    if theta1 > theta2
        theta2 = theta2 + 2*pi
    end

    angle = theta2 - theta1

    area = 0.5*circle.R^2*angle

    return area
end


# Vector: Any length vector, with each row of form [[Association_Object(s)], Any_Other_Objects]
# Returns a vector, with each row containing rows in the original Vector that have links between the Association Objects
function associate(Vector)
    global dummy = []
    push!(dummy,Vector[1])
    splice!(Vector,1)

    final = []

    if size(Vector)[1] == 0
        push!(final,dummy)
    else
        while size(Vector)[1] != 0
            super_break = false

            for i in range(1,stop=size(dummy)[1])
                for j in range(1,stop=size(Vector)[1])
                    var1 = dummy[i][1]
                    var2 = Vector[j][1]

                    common = intersect(var1,var2)

                    if size(common)[1] != 0 # there exists a common object
                        push!(dummy,Vector[j])
                        splice!(Vector,j)
                        super_break = true
                        break
                    end

                    if i == size(dummy)[1] && j == size(Vector)[1] # no more common objects between original vector and vector of associated objects
                        push!(final,dummy)
                        global dummy = []
                        push!(dummy,Vector[1])
                        splice!(Vector,1)
                    end
                    
                end

                if super_break
                    break
                end
            end

            if size(Vector)[1] == 0
                push!(final,dummy)
            end

        end

    end

    return final
end

# Function for forming >4 sided polygons. "vector" is a Vector of all the points, and their associated
# circles, that form one or more >4 sided polygons.
#
# Each row in "vector" is of the form [[circle1,circle2], [point]], where circle1 and circle2 are the circles that intersect
# to form point.
#
# In the 'associate' function above, we start at the first point (first row of 'vector') and iterate through 'vector', looking for 
# another point that shares an associated circle with the first point. Once found, we repeat this process again, looking for an 
# associated circle with the second point, and so on.
#
# The key problem with this is that if we have 2 >4 sided polygons that share a common circle, the function groups them together as 
# one big >4 sided polygon. This will produce errors.
# 
# Our 'associate2' function works around this problem by taking advantage of the fact that for any single polygon, a circle
# can only be used to link 2 points, and only 2 points, together.
#
# Hence, we can iterate through 'vector' as before. While iterating, we store the circles that have already been visited. So we 
# only look for points that are associated with a circle not already visited. We also know that the last point in a polygon has 
# to be associated with one of the circles associated with the first point. In particular, this circle is the circle that was not 
# used to find the second point in the polygon.
# 
# Once we cannot find any more associated points, we check if the last point is associated with the first point through the circle 
# that was not used to find the second point from the first point. If the circles do not match, we shuffle our vector and try again.
function associate2(vector)
    myvect = copy(vector)
    dummy = []
    visited = []
    last_circle = []
    final = []

    push!(dummy, myvect[1])
    splice!(myvect, 1)

    while size(myvect)[1] > 0   # we iterate through myvect until all points have been assigned to a polygon
        end_of_iterations = true    # marker to signal that all points for a particular polygon have been put into dummy
        for i in range(1,stop=size(myvect)[1])
            var1 = last(dummy)
            var2 = myvect[i]

            common = intersect(var1[1], var2[1]) # the common circle between 2 points

            # a common circle exists that hasn't been previously visited
            if size(common)[1] > 0 && (common[1] in visited) == false
                if length(dummy) == 1 # second polygon point found, we now know what the associated circle for the last point is
                    last_circle_index = findall(x -> x != common[1], var1[1])
                    push!(last_circle, var1[1][last_circle_index][1])
                end
                push!(dummy,var2) # push this point to dummy
                splice!(myvect,i) # remove this point from myvect
                push!(visited, common[1]) # add the common circle to vector of visited circles
                end_of_iterations = false
                break
            end
        end

        # check if the last point in dummy shares the same circle as what we know the last point should
        supposed_last = last(dummy)[1][findall(x -> x != last(visited), last(dummy)[1])[1]]
        if supposed_last == last_circle[1] # if last point circle matches, polygon is formed
            push!(final, dummy)
            vector = myvect;

            if size(myvect)[1] == 0
                break
            end

            dummy = []
            visited = []
            last_circle = []

            push!(dummy, myvect[1])
            splice!(myvect, 1)
        elseif end_of_iterations
            vector = shuffle(vector)
            myvect = copy(vector)

            dummy = []
            visited = []
            last_circle = []

            push!(dummy, myvect[1])
            splice!(myvect, 1)
        end
    end

    return final
end

end #module end