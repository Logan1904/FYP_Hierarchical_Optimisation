module Greens_Method

export Greens_Problem

using Base_Functions

"""
    struct Greens_Problem

An object describing the problem of finding the area of the union of N intersecting circles
The problem is solved by obtaining the outer contours of the circles, and calculating the area encompassed by Greens Theorem

Attributes:
    - 'circles::Vector{Circle}':                            A vector of Circle objects describing our problem
    - 'intersections::Vector{Point}':                       A vector of Point objects describing the intersection points ONLY LOCATED ON AN OUTER CONTOUR
    - 'contour::Vector{Vector{Circle}}':                    A vector of Circle objects, where each inner vector describes the Circle objects that are 'associated' with one another
    - 'boundary::Vector{Vector{Vector{Vector{Any}}}}':      A vector of vector of vectors describing the outer boundaries
    - 'area::Float64':                                      The calculated area
"""
struct Greens_Problem
    circles::Vector{Circle}
    intersections::Vector{Point}
    contours::Vector{Vector{Circle}}
    boundaries::Vector{Vector{Vector{Any}}}
    area::Float64

    function Greens_Problem(x::Vector{Float64})
        circles = make_circles(x)

        original_circles = copy(circles)

        circles = checkCoincident!(circles)
        circles = checkContained!(circles)
        intersections = getIntersections(circles)
        contours = getContours(circles)
        boundaries = getBoundaries(circles,intersections,contours)
        area = calculateArea(boundaries)

        new(original_circles,intersections,contours,boundaries,area)
    end

end

"""
    checkCoincident!(circles::Vector{Circle})

Checks if any Circle objects are coincident with each other and removes the redundant Circle objects

# Arguments:

    - 'circles::Vector{Circle}': Vector of Circle objects
"""
function checkCoincident!(circles::Vector{Circle})
    clean = false
    while !clean
        N = Int(length(circles))
        clean = true
        for i in 1:N, j in i+1:N
            A = circles[i]
            B = circles[j]

            if coincident(A,B)
                clean = false
                deleteat!(circles,j)
                break
            end
        end
    end

    return circles
end

"""
    checkContained!(circles::Vector{Circle})

Checks if any Circle objects are contained within one another and removes the contained Circle objects

# Arguments:

    - 'circles::Vector{Circle}': Vector of Circle objects
"""
function checkContained!(circles::Vector{Circle})
    for i in 1:length(circles)
        for j in i+1:length(circles)
            contained(circles[i],circles[j])
        end
    end

    new_circles = [i for i in circles if i.Contained == false]

    return new_circles
end

"""
    getIntersections(circles::Vector{Circle})


Obtains the intersection Point objects only located on an outer contour

Updates the attributes of the Circle objects in 'circles' accordingly

Returns intersections::Vector{Point} containing the Point objects

# Arguments:

    - 'circles::Vector{Circle}': Vector of Circle objects
"""
function getIntersections(circles::Vector{Circle})
    intersections = Vector{Point}()
    intersections_circles = []
    for i in 1:length(circles)

        for j in i+1:length(circles)
            A = circles[i]
            B = circles[j]

            coords = intersection(circles[i],circles[j])

            if isnothing(coords) == false
                # update the Circles attribute of the Circle objects
                push!(A.Circles,i,j)
                push!(B.Circles,i,j)

                x1,y1,x2,y2 = coords

                # form the Point objects representing the intersection points
                P1 = Point(x1,y1,[i,j],1)
                P2 = Point(x2,y2,[i,j],1)

                # check if P1 and P2 are outer contour points
                for P in [P1,P2]
                    contour_point = true
                    for k in 1:length(circles)
                        if outside(circles[k],P) == false
                            contour_point = false
                            break
                        end
                    end
                    
                    if contour_point
                        push!(intersections,P)

                        # update the Points attribute of the Circle objects
                        dum = length(intersections)
                        push!(A.Points,dum)
                        push!(B.Points,dum)

                    end
                end
                
            end
        end
    end

    # Ensure no repetitions in the Circle attribute of the Circle objects
    for i in 1:length(circles)
        circles[i].Circles = unique(circles[i].Circles)
    end

    return intersections
end

"""
    getContours(circles::Vector{Circle})

Obtains the contours of our problem

Returns contour::Vector{Vector{Circle}} where each inner vector contains the Circle objects that make up a contour

# Arguments:

    - 'circles::Vector{Circle}': Vector of Circle objects
"""
function getContours(circles)# form circle associations

    # if there is just one Circle
    if length(circles) == 1
        return [circles]
    end

    # form input format for associate function
    contours = []
    contour_circles = copy(circles)
    for i in range(1,stop=length(contour_circles))
        A = contour_circles[i]
        push!(contours,[A.Circles,A])
    end

    contours = associate(contours)

    # change into vectors of Circles
    for i in 1:length(contours)
        contours[i] = [point[2] for point in contours[i]]
    end

    return contours
end

"""
    getBoundaries(circles::Vector{Circle}, intersections::Vector{Point}, contours::Vector{Vector{Circle}})


Obtains the outer contours of our problem

Returns contour::Vector{Vector{Circle}} where each inner vector contains the Circle objects that make up a contour

# Arguments:

    - 'circles::Vector{Circle}': Vector of Circle objects
"""
function getBoundaries(circles,intersections,contours)
    boundaries = []
    for i in range(1,stop=size(contours)[1])
        contour_boundaries = []

        # just the circle
        if size(contours[i])[1] == 1 
            push!(contour_boundaries,[[contours[i][1],Point(0,0,[],0),Point(0,0,[],0),0.0,2*pi]])
            push!(boundaries,contour_boundaries)
            continue
        end

        # obtain the boundaries for each contour
        for j in 1:length(contours[i]) 
            circle = contours[i][j]
            boundary_points = intersections[circle.Points]

            sort_asc_angle!(circle,boundary_points)
            theta = [mod(atan(point.y-circle.y,point.x-circle.x),2*pi) for point in boundary_points]

            for k in range(1,stop=length(theta))
                next_iterator = mod1(k+1,length(theta))

                if k == length(theta)
                    half_angle = 0.5*(theta[k]+theta[next_iterator]) + pi
                else
                    half_angle = 0.5*(theta[k]+theta[next_iterator])
                end

                point_check = point_on_circle(circle,half_angle)

                A = circles[circle.Circles]

                # check if point_check is inside any of the circles that intersect our current circle
                for l in range(1,stop=length(A)) 
                    if outside(A[l],point_check) == false
                        break
                    elseif l == length(A)
                        push!(contour_boundaries,[circle,boundary_points[k],boundary_points[next_iterator],theta[k],theta[next_iterator]])
                    end
                end

            end

        end

        # sort the boundaries so they form a closed curve
        for j in 1:length(contour_boundaries)-1
            end_sec1 = contour_boundaries[j][3]

            for k in j+1:length(contour_boundaries)
                start_sec2 = contour_boundaries[k][2]

                if end_sec1 == start_sec2
                    tmp = contour_boundaries[j+1]
                    contour_boundaries[j+1] = contour_boundaries[k]
                    contour_boundaries[k] = tmp
                    break
                end
            end
        end

        # split the boundaries so each boundary describes a single closed curve
        tmp = []
        while length(contour_boundaries) > 1
            for j in 1:length(contour_boundaries)-1

                end_sec1 = contour_boundaries[j][3]
                start_sec2 = contour_boundaries[j+1][2]

                # check for discontinuity
                if end_sec1 != start_sec2
                    push!(tmp,contour_boundaries[1:j])
                    deleteat!(contour_boundaries, 1:j)
                    break
                end

                if j == length(contour_boundaries)-1
                    push!(tmp,contour_boundaries)
                    contour_boundaries = []
                end
            end
        end

        push!(boundaries,tmp)
    end

    return boundaries
end

function calculateArea(boundaries)

    total_area = 0

    # for each collection
    for i in 1:length(boundaries)

        coll_area = []

        # for each contour
        for j in 1:length(boundaries[i])
            
            if length(boundaries[i][j]) == 1
                push!(coll_area,pi*(boundaries[i][1][1][1].R)^2)
                break
            end

            vec = Vector{Point}()

            for k in 1:length(boundaries[i][j])

                circle = boundaries[i][j][k][1]
                theta1 = boundaries[i][j][k][4]
                theta2 = boundaries[i][j][k][5]

                if theta1 > theta2
                    theta2 = theta2 + 2*pi
                end

                angles = LinRange(theta1,theta2,101)

                for l in angles

                    x = circle.x + circle.R*cos(l)
                    y = circle.y + circle.R*sin(l)

                    P = Point(x,y,[],1)
                    push!(vec,P)

                end

            end

            push!(coll_area, shoelace(vec))

        end

        if length(coll_area) == 1
            total_area += coll_area[1]
        else
            var,ind = findmax(coll_area)
            deleteat!(coll_area,ind)

            total_area += var - sum(coll_area)
        end

    end

    return total_area

end

end #module end