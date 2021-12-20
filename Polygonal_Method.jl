N = 6

circles = []
for i in range(1,stop=N)
    x,y = rand(-10:10,2)
    R = rand(2:5,1)[1]

    push!(circles,Functions.Circle(x,y,R,[]))
end

circles = [Functions.Circle(-1,0,1,[]),Functions.Circle(1,0,1,[]),Functions.Circle(0,1,1,[]),Functions.Circle(0,-0.5,1,[]),
Functions.Circle(-1,-2.5,1,[]),Functions.Circle(1,-2.5,1,[]),Functions.Circle(0,-2,1,[]),Functions.Circle(0,-3,1,[])]

intersections = []

#get intersection points for all circles
for i in range(1,stop=length(circles))
    for j in range(i+1,stop=length(circles))
        coords = Functions.intersection(circles[i],circles[j])

        if isnothing(coords) == false
            x1,y1,x2,y2 = coords
            A = circles[i]
            B = circles[j]

            P1 = Functions.Point(x1,y1,A,B,1)
            P2 = Functions.Point(x2,y2,A,B,1)
            push!(intersections,P1)
            push!(intersections,P2)

            dum = size(intersections)[1]

            push!(A.Points,dum-1,dum)
            push!(B.Points,dum-1,dum)
        end

    end
end

#obtain boundary ID -> 1: on outer contour, 0: contained inside contour
for i in range(1,stop=size(intersections)[1])
    point = intersections[i]

    for j in range(1,stop=length(circles))
        circle = circles[j]
        
        ID = Functions.boundary(circle,point)

        if ID == 0
            point.ID = ID
            break
        else
            continue
        end

    end
end

#form polygons
polygon = []
big_polygon = []
for i in range(1,stop=length(circles))
    for j in range(i+1,stop=length(circles))
        shared_points_index = intersect(circles[i].Points, circles[j].Points)

        storage = []
        for k in range(1,stop=size(shared_points_index)[1])
            point = intersections[shared_points_index[k]]

            if point.ID == 0
                push!(storage,k) # store index of shared point (only non-contour point) between 2 circles
            end

        end

        splice!(shared_points_index,storage) # obtain index of shared points that are contour points

        if size(shared_points_index)[1] == 0 # no shared contour points
            continue
        elseif size(shared_points_index)[1] == 2 # 2 shared contour points -> a 4 sided polygon
            push!(polygon,[circles[i],circles[j],intersections[shared_points_index[1]],intersections[shared_points_index[2]]])
        elseif size(shared_points_index)[1] == 1 # 1 shared contour point -> part of a bigger >4 sided polygon
            push!(big_polygon,[[circles[i],circles[j]],intersections[shared_points_index[1]]])
        end
        
    end
end

# form big polygon points
if size(big_polygon)[1] != 0

    big_polygon = Functions.associate(big_polygon)

    #unpack and form unique points of big polygon
    for i in range(1,stop=size(big_polygon)[1])
        storage = [] 
        for j in range(1,stop=size(big_polygon[i])[1])
            A,B = big_polygon[i][j][1]
            point = big_polygon[i][j][2]
            push!(storage,A,B,point)
        end

        storage = unique(storage)

        push!(polygon,storage)
    end
end

# sort polygon points ACW and obtain area
area = 0
for i in range(1,stop=size(polygon)[1])
    polygon[i] = Functions.sort(polygon[i])
    global area = area + Functions.shoelace(polygon[i])
end

"""
# form contour points
boundary_points = [intersections[i] for i in range(1,stop=size(intersections)[1]) if intersections[i].ID == 1]

contour = []
for i in range(1,stop=size(boundary_points)[1])
    point = boundary_points[i]
    A = point.A
    B = point.B

    push!(contour,[A,B,point])
end

contour = Functions.associate(contour)

# change into just the point
for i in range(1,stop=size(contour)[1])
    storage = []
    for j in range(1,stop=size(contour[i])[1])
        point = contour[i][j][3]
        push!(storage,point)
    end

    contour[i] = storage
end

# sort points
for i in range(1,stop=size(contour)[1])
    contour[i] = Functions.sort(contour[i])
end

sectors = []
for i in range(1,stop=size(contour)[1])
    for j in range(1,stop=size(contour[i])[1])
        next_iterator = mod1(j+1,size(contour[i])[1])
        prev_iterator = mod1(j-1,size(contour[i])[1])

        point1 = contour[i][j]
        point2 = contour[i][next_iterator]
        point3 = contour[i][prev_iterator]

        A1 = point1.A
        B1 = point1.B
        A2 = point2.A
        B2 = point2.B
        A3 = point3.A
        B3 = point3.B
    
        common_circles = intersect([A1,B1],[A2,B2])

        if size(common_circles)[1] == 1
            circle = common_circles[1]
        elseif size(common_circles)[1] == 2
            prev_common_circle = intersect([A1,B1],[A3,B3])[1]
            circle = setdiff(common_circles,prev_common_circle)[1]
        end

        theta1 = mod(atan(point1.y-circle.y,point1.x-circle.x),2*pi)
        theta2 = mod(atan(point2.y-circle.y,point2.x-circle.x),2*pi)

        if theta1 < theta2
            angle = theta2 - theta1
        else
            angle = 2*pi - (theta1-theta2)
        end
        
        global area = area + 0.5*angle*(common_circles[1].R)^2
        push!(sectors,[circle,theta1,theta2])
    end
end
    

sectors = []
for i in range(1,stop=size(contour)[1])
    for j in range(1,stop=size(contour[i])[1])



    next_iterator = mod1(i+1,size(contour)[1])
    
    point1 = contour[i]
    point2 = contour[next_iterator]

    A1 = point1.A
    B1 = point1.B
    A2 = point2.A
    B2 = point2.B

    common_circles = intersect([A1,B1],[A2,B2])

    if size(common_circles)[1] == 1
        circle = common_circles[1]
        theta1 = mod(atan(point1.y-circle.y,point1.x-circle.x),2*pi)
        theta2 = mod(atan(point2.y-circle.y,point2.x-circle.x),2*pi)

        if theta1 < theta2
            angle = theta2 - theta1
        else
            angle = 2*pi - (theta1-theta2)
        end
        
        global area = area + 0.5*angle*(common_circles[1].R)^2
        push!(sectors,[circle,theta1,theta2])
    else

    end
end
"""