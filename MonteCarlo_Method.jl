module MonteCarlo_Method
import Functions

function Area(arr,N_MC,domain)
    circles = Functions.make_circles(arr)
    N_MC = 100000
    count = 0
    domain_x,domain_y = domain
    for i in range(1,stop=N_MC)
        point_x = rand(1)[1]*(domain_x)
        point_y = rand(1)[1]*(domain_y)

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

    area = count/N_MC * (domain_x*domain_y)
    println("Total Area (MonteCarlo): ",area)
    return area
end

end #module end