module Plotter
import Functions
using Plots

function plot_domain(circles,domain,string)
    p = plot()
    domain_x,domain_y = domain
    
    # plot circles
    for i in range(1,stop=length(circles))
        plot!(Functions.draw(circles[i],0.0,2*pi), aspect_ratio=1, label="", color="black", ylim=(0,domain_y), xlim=(0,domain_x), legend=:outertopright)
    end

    savefig(string)
end

function plot_all(circles, intersections, polygons, sectors, domain, string)
    p = plot()
    domain_x,domain_y = domain
    #plot circles
    for i in range(1,stop=length(circles))
        plot!(Functions.draw(circles[i],0.0,2*pi), aspect_ratio=1, label="", color="black", ylim=(0,domain_y), xlim=(0,domain_x), legend=:outertopright)
    end

    #plot intersection points
    for i in range(1,stop=length(intersections))
        point = intersections[i]
        if point.ID == 1
            color = "red"
            plot!([point.x],[point.y], seriestype=:scatter, label="", color=color)
        end

        
    end

    #plot polygons
    for i in range(1,stop=size(polygons)[1])
        xarr = [point.x for point in polygons[i]]
        yarr = [point.y for point in polygons[i]]

        append!(xarr,xarr[1])
        append!(yarr,yarr[1])

        plot!(xarr,yarr,color="blue", label="")
    end

    #plot circular sectors
    for i in range(1,stop=size(sectors)[1])
        plot!(Functions.draw(sectors[i][1],sectors[i][2],sectors[i][3]), label="", color="red")
    end

    savefig(string)
end

end #module end