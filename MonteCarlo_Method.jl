module MonteCarlo_Method
import Functions

function Area(arr,N_MC; print::Bool=false)
    circles = Functions.make_circles(arr)

    max_x = max([point.x + point.R for point in circles]...)
    max_y = max([point.y + point.R for point in circles]...)
    min_x = min([point.x - point.R for point in circles]...)
    min_y = min([point.y - point.R for point in circles]...)

    count = 0
    for i in range(1,stop=N_MC)
        point_x = rand(1)[1]*(max_x-min_x)+min_x
        point_y = rand(1)[1]*(max_y-min_y)+min_y

        Point = Functions.Point(point_x,point_y,[],false)

        for j in range(1,stop=size(circles)[1])
            A = circles[j]
            inside = Functions.boundary(A,Point)

            if !inside #point inside a circle
                count = count + 1
                break
            end
        end

    end

    area = count/N_MC * ((max_x-min_x)*(max_y-min_y))
    if print
        println("Total Area (MonteCarlo): ",area)
    end
    return area
end

end #module end