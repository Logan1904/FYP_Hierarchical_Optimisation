module Polygonal_Method

import Functions

using Graphs

function Area(Arr; return_objects::Bool=false, return_graph::Bool=false)
    circles = Functions.make_circles(Arr)
    check_coincident!(circles)                              #check if any circles are coincident
    intersections = intersection_points(circles)            #get intersection points for all circles
    boundary_ID!(circles,intersections)                     #obtain boundary ID -> 1: on outer contour, 0: contained inside contour
    polygons = form_polygons(circles,intersections)         #form polygons
    sort_polygon_acw!(circles,polygons)                     #sort polygons acw
    contour = form_contour(circles)                         #form contour
    sectors = form_sectors(circles,intersections,contour)   #form sectors

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

function intersection_points(circles::Vector{Functions.Circle})
    N = Int(length(circles))

    intersections = Vector{Functions.Point}()
    for i in 1:N
        for j in i+1:N
            coords = Functions.intersection(circles[i],circles[j])

            if isnothing(coords) == false
                x1,y1,x2,y2 = coords
                A = circles[i]
                B = circles[j]

                P1 = Functions.Point(x1,y1,[i,j],1)
                P2 = Functions.Point(x2,y2,[i,j],1)
                push!(intersections,P1)
                push!(intersections,P2)

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

function form_polygons_graph(circles, intersections)
    # for each point in intersections, get points only on outer contour
    contour_points = [point for point in intersections if point.ID == 1]

    # check if there are >1 contour points at the same x,y coordinate
    # if so, combine them together into one point
    dirty = true
    while dirty
        dirty = false
        super_break = false
        for i in 1:length(contour_points)
            for j in i+1:length(contour_points)
                point1 = contour_points[i]
                point2 = contour_points[j]

                if isapprox(point1.x,point2.x,atol=0.1) && isapprox(point1.y,point2.y,atol=0.1)
                    deleteat!(contour_points, [i,j]) # delete old points with same x,y coords formed from different circles
                    new_circles = unique(vcat(point1.Circles, point2.Circles)) # get the circles that intersect on the point, no repeated circles
                    new_point = Functions.Point(point1.x, point1.y, new_circles, 1)
                    push!(contour_points, new_point) # add this new point to our vector of contour points
                    dirty = true
                    super_break = true
                    break
                end
            end

            if super_break
                break
            end

        end
    end

    # make a simple undirected graph using Graphs.jl
    # nodes are contour points + every circle that make up those contour points
    # (remember that a point is made up of 2 or more circles that intersect at that point)
    # edges are connections between a point and the circles that make up that point

    # get a vector of all the circles that make up the contour points, removing repeated circles
    tmp = [point.Circles for point in contour_points]
    unique_circles = []
    for i in 1:length(tmp)
        for j in 1:length(tmp[i])
            push!(unique_circles, tmp[i][j])
        end
    end

    unique!(unique_circles)

    # our vector of nodes
    nodes = vcat(contour_points, circles[unique_circles])

    g =SimpleGraph(length(nodes))

    for i in 1:length(contour_points)
        point_circle_index = contour_points[i].Circles # the circles that make up this contour point
        for j in 1:length(unique_circles)
            circle_index = unique_circles[j]

            if circle_index in point_circle_index
                add_edge!(g, i, j+length(contour_points)) # form edge
            end
        end
    end

    f = copy(g)
    
    special_nodes = []
    for i in nv(g)-length(circles)+1:nv(g)
        n_edges = degree(g,i)
        if n_edges > 2 && n_edges %2 == 0
            push!(special_nodes,i)
        end
    end
    
    poly = []
    for i in 1:length(special_nodes)
        for j in i+1:length(special_nodes)
            node1 = special_nodes[i]
            node2 = special_nodes[j]
    
            paths = yen_k_shortest_paths(g,node1,node2,weights(g),2).paths
            if length(paths) == 0
                continue
            end
            check = union(paths[1][2:end-1],paths[2][2:end-1])
    
            super_break = false
            for k in 1:length(check)
                if check[k] in special_nodes
                    #println("Node ",node1," to node ",node2," is invalid")
                    super_break = true
                    break
                end
            end
    
            if super_break
                continue
            end
    
            push!(poly,paths)
    
            for vertex in check
                rem_edge!(g,node1,vertex)
                rem_edge!(g,node2,vertex)
            end
    
        end
    end
    
    for i in 1:length(poly)
        poly[i] = union(poly[i][1],poly[i][2])
    end
    
    poly = vcat(poly,cycle_basis(g))

    # get the minimal collection of cycles
    # a cycle is essentially a polygon
    #cycles = cycle_basis(g)

    polygons = [nodes[i] for i in poly] #put the corresponding point and circle objects into polygons vector

    return polygons, f
end

function form_polygons(circles::Vector{Functions.Circle},intersections::Vector{Functions.Point})
    N = Int(length(circles))

    polygon = Vector{Any}()
    big_polygon = Vector{Any}()
    for i in 1:N
        for j in i+1:N
            A = circles[i]
            B = circles[j]

            # if either circle is completely within another circle, ignore
            if A.Contained == 1 || B.Contained == 1
                continue
            end

            # obtain indices of shared points between 2 circles
            shared_points_index = intersect(A.Points, B.Points)
            
            # now obtain shared points between 2 circles only if the point is on contour boundary
            shared_points = [intersections[k] for k in shared_points_index if intersections[k].ID == 1]

            if length(shared_points) == 0                                       # no shared contour points
                continue
        elseif length(shared_points) == 2                                       # 2 shared contour points -> a 4 sided polygon
                push!(polygon, [A, B, shared_points[1], shared_points[2]])
        elseif length(shared_points) == 1                                       # 1 shared contour point -> part of a bigger >4 sided polygon
                push!(big_polygon, [[A, B], shared_points[1]])
            end
            
        end
    end

    # form big polygon points
    if size(big_polygon)[1] != 0

        big_polygon = Functions.associate2(big_polygon)

        #unpack and form unique points of big polygon
        for i in 1:size(big_polygon)[1]
            storage = [] 
            for j in 1:size(big_polygon[i])[1]
                A,B = big_polygon[i][j][1]
                point = big_polygon[i][j][2]
                push!(storage,A,B,point)
            end

            storage = unique(storage)

            push!(polygon,storage)
        end
    end

    return polygon
end

function sort_polygon_acw!(circles,polygon)
    for i in 1:length(polygon)
        if size(polygon[i])[1] == 4                                                             # for 4 sided polygon, sort acw with centre as the average of the 2 circular centres
            dummy = [point for point in polygon[i] if isa(point,Functions.Circle) == true]
            mean_x = sum([point.x for point in dummy])/2
            mean_y = sum([point.y for point in dummy])/2

            Functions.sort_acw!(polygon[i],mean_x,mean_y)
        else                                                                                    # for >4 sided polygon, sort acw/cw by jumping between points and seeing which circle is shared

            if typeof(polygon[i][1]) == Functions.Circle
                polygon[i] = circshift(polygon[i],1)
            end

            dummy = []
            A = [point for point in polygon[i] if typeof(point)==Functions.Point]

            push!(dummy,A[1])
            splice!(A,1)

            while size(A)[1] != 0
                for j in range(1,stop=size(A)[1])
                    point = A[j]
                    prev_point = last(dummy)

                    point_circles = circles[point.Circles]
                    prev_point_circles = circles[prev_point.Circles]

                    common_circle = intersect(point_circles,prev_point_circles)

                    if size(common_circle)[1] != 0
                        push!(dummy,common_circle[1])
                        push!(dummy,point)
                        splice!(A,j)
                        break
                    end
                end
            end

            first_point_circles = circles[dummy[1].Circles]
            last_point_circles = circles[last(dummy).Circles]

            common_circle = intersect(first_point_circles,last_point_circles)

            push!(dummy,common_circle[1])
            polygon[i] = dummy
        end
    end
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
            index = circle.Points
            boundary_points = [intersections[x] for x in index if intersections[x].ID == 1]

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

end #module end