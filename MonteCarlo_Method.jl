module MonteCarlo_Method
import Functions

function Area(arr,N_MC; print::Bool=false)
    circles = Functions.make_circles(arr)

    max_x = circles[1].x + circles[1].R
    min_x = circles[1].x - circles[1].R
    max_y = circles[1].y + circles[1].R
    min_y = circles[1].y - circles[1].R
    for i in 2:length(circles)
        A = circles[i]
        x = A.x
        y = A.y
        R = A.R

        if x+R > max_x
            max_x = x+R
        end

        if x-R < min_x
            min_x = x-R
        end

        if y+R > max_y
            max_y = y+R
        end

        if y-R < min_y
            min_y = y-R
        end
    end

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