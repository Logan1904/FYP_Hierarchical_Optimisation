module Polygonal_Method

import Functions

using Graphs

function Area(Arr; return_objects::Bool=false, return_graph::Bool=false)
    circles = Functions.make_circles(Arr)

    Polygonal_Method.check_coincident!(circles)
    
    circles = Polygonal_Method.check_contained!(circles)
    
    Polygonal_Method.check_partially_contained!(circles)
    
    intersections = Polygonal_Method.intersection_points(circles)
    
    Polygonal_Method.boundary_ID!(circles,intersections)
    
    intersections = Polygonal_Method.form_contour_points!(circles,intersections)
    
    contour = Polygonal_Method.form_contour(circles)
    
    sectors = Polygonal_Method.form_sectors(circles,intersections,contour)
    
    g, f, polygons, nodes = Polygonal_Method.form_polygons(circles,intersections,contour)
    
    area = 0
    for i in 1:length(polygons)                             #obtain area of polygons
        var = Functions.shoelace(polygons[i])
        area = area + var
    end

    for i in 1:length(sectors)                              #obtain area of sectors
        A,Theta1,Theta2 = sectors[i]
        var = Functions.area_sector(A,Theta1,Theta2)
        area = area + var
    end

    if return_objects
        return area, circles, intersections, polygons, sectors
    end

    if return_graph
        return area, circles, intersections, polygons, sectors, g
    end

    return area

end

function check_coincident!(circles::Vector{Functions.Circle})
    clean = false
    while !clean
        N = Int(length(circles))
        clean = true
        for i in 1:N, j in i+1:N
            A = circles[i]
            B = circles[j]

            if Functions.coincident(A,B)
                clean = false
                deleteat!(circles,j)
                break
            end
        end
    end
end

function check_contained!(circles::Vector{Functions.Circle})

    for i in 1:length(circles)
        for j in i+1:length(circles)
            Functions.contained(circles[i],circles[j])
        end
    end

    new_circles = [i for i in circles if i.Contained == false]

    return new_circles
end

function check_partially_contained!(circles::Vector{Functions.Circle})

    dirty = true
    while dirty

        for i in 1:length(circles)
            points = Vector{Functions.Point}()

            for j in 1:length(circles)
                coords = Functions.intersection(circles[i],circles[j])

                if isnothing(coords) == false

                    x1,y1,x2,y2 = coords

                    P1 = Functions.Point(x1,y1,[i,j],1)
                    P2 = Functions.Point(x2,y2,[i,j],1)

                    push!(points,P1,P2)
                end
            end

            point_ID = ones(Bool, length(points))
            for k in 1:length(points)
                P = points[k]

                for l in 1:length(circles)

                    ID = Functions.boundary(circles[l],P)

                    if ID == false
                        point_ID[k] = false
                        break
                    end
                end
                
            end

            if all(!,point_ID) && length(points) > 0
                deleteat!(circles,i)
                dirty = true
                break
            end

            dirty = false
        end
    end
end

function intersection_points(circles::Vector{Functions.Circle})
    intersections = Vector{Functions.Point}()
    for i in 1:length(circles)

        for j in i+1:length(circles)
            A = circles[i]
            B = circles[j]

            coords = Functions.intersection(circles[i],circles[j])

            if isnothing(coords) == false

                x1,y1,x2,y2 = coords

                P1 = Functions.Point(x1,y1,[i,j],1)
                P2 = Functions.Point(x2,y2,[i,j],1)

                push!(intersections,P1,P2)

                dum = Int(length(intersections))

                push!(A.Points,dum-1,dum)
                push!(B.Points,dum-1,dum)
                
                push!(A.Circles,j)
                push!(B.Circles,i)
                
            end
        end
    end

    return intersections
end

function boundary_ID!(circles::Vector{Functions.Circle},intersections::Vector{Functions.Point})
    for i in 1:Int(length(intersections))
        P = intersections[i]

        for j in 1:Int(length(circles))
            A = circles[j]
            
            ID = Functions.boundary(A,P)

            if ID == 0
                P.ID = ID
                break
            else
                continue
            end

        end
    end
end

function form_contour_points!(circles, intersections)

    contour_points = [point for point in intersections if point.ID==1]

    for i in 1:size(circles)[1]
        circles[i].Points = []
    end

    for i in 1:size(contour_points)[1]
        for j in contour_points[i].Circles
            push!(circles[j].Points,i)
        end
    end

    return contour_points
end

function form_contour(circles)# form circle associations
    contour = []
    contour_circles = [point for point in circles if point.Contained != true]
    for i in range(1,stop=length(contour_circles))
        A = contour_circles[i]
        push!(contour,[A.Points,A])
    end

    contour = Functions.associate(contour)

    # change into vectors of circles
    for i in range(1,stop=size(contour)[1])
        contour[i] = [point[2] for point in contour[i]]
    end

    return contour
end

function form_sectors(circles,intersections,contour)# form circular sectors
    sectors = []
    for i in range(1,stop=size(contour)[1])

        if size(contour[i])[1] == 1 # just the circle
            push!(sectors,[contour[i][1],0.0,2*pi])
            continue
        end

        for j in range(1,stop=size(contour[i])[1]) # obtain circular sectors for each circle 
            circle = contour[i][j]
            boundary_points = intersections[circle.Points]

            Functions.sort_asc_angle!(circle,boundary_points)
            theta = [mod(atan(point.y-circle.y,point.x-circle.x),2*pi) for point in boundary_points]

            for k in range(1,stop=length(theta))
                next_iterator = mod1(k+1,length(theta))

                if k == length(theta)
                    half_angle = 0.5*(theta[k]+theta[next_iterator]) + pi
                else
                    half_angle = 0.5*(theta[k]+theta[next_iterator])
                end

                point_check = Functions.point_on_circle(circle,half_angle)

                A = circles[circle.Circles]

                for l in range(1,stop=length(A)) # check if point_check is inside any of the circles that intersect our current circle
                    outside = Functions.boundary(A[l],point_check)
                    if outside == 0
                        break
                    elseif l == length(A)
                        push!(sectors,[circle,theta[k],theta[next_iterator]])
                    end
                end

            end

        end
    end

    return sectors
end

function form_polygons(circles, intersections, contour)

    polygon = []

    nodes = vcat(circles,intersections)
    g =SimpleGraph(length(nodes))

    ind = []
    ind_circles = []
    # if 2 points have the same circles -> simple polygon
    for i in 1:length(intersections)
        for j in i+1:length(intersections)
            P1 = intersections[i]
            P2 = intersections[j]

            if P1.Circles == P2.Circles
                push!(ind,[i,j])
                push!(ind_circles,P1.Circles)
            end
        end
    end

    for i in 1:length(ind)
        push!(polygon,[ind[i][1]+length(circles),ind_circles[i][1],ind_circles[i][2]])
        push!(polygon,[ind[i][2]+length(circles),ind_circles[i][1],ind_circles[i][2]])
    end

    ind = reduce(vcat,ind)
    ind_circles = reduce(vcat,ind_circles)

    # edges between each point and its 2 circles
    for i in 1:length(intersections)

        if i in ind
            continue
        end

        A,B = intersections[i].Circles

        add_edge!(g,i+length(circles),A)
        add_edge!(g,i+length(circles),B)
        add_edge!(g,A,B)
    end

    f = copy(g)

    dirty = true
    while dirty
        dirty = false

        for i in 1:length(nodes)

            d = degree(g,i)

            if d == 2
                n1,n2 = neighbors(g,i)

                if has_edge(g,n1,n2) == true
                    push!(polygon,[i,n1,n2])
                    rem_edge!(g,i,n1)
                    rem_edge!(g,i,n2)
                    dirty = true
                    break
                end

            end

        end

    end

    polygon = [nodes[i] for i in polygon]

    return g, f, polygon, nodes

end

end #module end