module Greens_Method

export Greens_Problem

using Base_Functions

struct Greens_Problem
    circles::Vector{Circle}
    intersections::Vector{Point}
    contour::Vector{Vector{Circle}}
    sectors::Vector{Vector{Vector{Any}}}
    area::Float64

    function Greens_Problem(x::Vector{Float64})
        circles = make_circles(x)

        original_circles = copy(circles)

        circles = check_coincident!(circles)
        circles = check_contained!(circles)
        intersections = intersection_points(circles)
        contour = form_contour(circles)
        sectors = form_sectors(circles,intersections,contour)
        area = calculateArea(sectors)

        new(original_circles,intersections,contour,sectors,area)
    end

end

function check_coincident!(circles::Vector{Circle})
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

function check_contained!(circles::Vector{Circle})
    for i in 1:length(circles)
        for j in i+1:length(circles)
            contained(circles[i],circles[j])
        end
    end

    new_circles = [i for i in circles if i.Contained == false]

    return new_circles
end


function intersection_points(circles::Vector{Circle})
    intersections = Vector{Point}()
    intersections_circles = []
    for i in 1:length(circles)

        for j in i+1:length(circles)
            A = circles[i]
            B = circles[j]

            coords = intersection(circles[i],circles[j])

            if isnothing(coords) == false
                push!(A.Circles,i,j)
                push!(B.Circles,i,j)

                x1,y1,x2,y2 = coords

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

                        dum = length(intersections)
                        push!(A.Points,dum)
                        push!(B.Points,dum)

                    end
                end
                
            end
        end
    end

    for i in 1:length(circles)
        circles[i].Circles = unique(circles[i].Circles)
    end

    return intersections
end

function form_contour(circles)# form circle associations

    if length(circles) == 1
        return [circles]
    end

    contour = []
    contour_circles = [circ for circ in circles]
    for i in range(1,stop=length(contour_circles))
        A = contour_circles[i]
        push!(contour,[A.Circles,A])
    end

    contour = associate(contour)

    # change into vectors of circles
    for i in range(1,stop=size(contour)[1])
        contour[i] = [point[2] for point in contour[i]]
    end

    return contour
end

function form_sectors(circles,intersections,contour)# form circular sectors
    sectors = []
    for i in range(1,stop=size(contour)[1])
        contour_sectors = []

        if size(contour[i])[1] == 1 # just the circle
            push!(contour_sectors,[[contour[i][1],Point(0,0,[],0),Point(0,0,[],0),0.0,2*pi]])
            push!(sectors,contour_sectors)
            continue
        end

        for j in range(1,stop=size(contour[i])[1]) # obtain circular sectors for each circle 
            circle = contour[i][j]
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

                for l in range(1,stop=length(A)) # check if point_check is inside any of the circles that intersect our current circle
                    if outside(A[l],point_check) == false
                        break
                    elseif l == length(A)
                        push!(contour_sectors,[circle,boundary_points[k],boundary_points[next_iterator],theta[k],theta[next_iterator]])
                    end
                end

            end

        end

        for j in 1:length(contour_sectors)-1
            end_sec1 = contour_sectors[j][3]

            for k in j+1:length(contour_sectors)
                start_sec2 = contour_sectors[k][2]

                if end_sec1 == start_sec2
                    tmp = contour_sectors[j+1]
                    contour_sectors[j+1] = contour_sectors[k]
                    contour_sectors[k] = tmp
                    break
                end
            end
        end

        tmp = []
        while length(contour_sectors) > 1
            for j in 1:length(contour_sectors)-1

                end_sec1 = contour_sectors[j][3]
                start_sec2 = contour_sectors[j+1][2]

                # check if there is a discontinuity in sectors
                if end_sec1 != start_sec2
                    push!(tmp,contour_sectors[1:j])
                    deleteat!(contour_sectors, 1:j)
                    break
                end

                if j == length(contour_sectors)-1
                    push!(tmp,contour_sectors)
                    contour_sectors = []
                end
            end
        end

        push!(sectors,tmp)

    end

    return sectors
end

function calculateArea(sectors)

    total_area = 0

    # for each collection
    for i in 1:length(sectors)

        coll_area = []

        # for each contour
        for j in 1:length(sectors[i])
            
            if length(sectors[i][j]) == 1
                push!(coll_area,pi*(sectors[i][1][1][1].R)^2)
                break
            end

            vec = Vector{Point}()

            for k in 1:length(sectors[i][j])

                circle = sectors[i][j][k][1]
                theta1 = sectors[i][j][k][4]
                theta2 = sectors[i][j][k][5]

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