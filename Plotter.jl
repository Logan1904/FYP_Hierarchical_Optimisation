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
    p = plot(minorgrid=true, minorgridalpha=0.25)
    domain_x,domain_y = domain
    #plot circles
    for i in range(1,stop=length(circles))
        plot!(Functions.draw(circles[i],0.0,2*pi), aspect_ratio=1, label="", color="black", ylim=(0,domain_y), xlim=(0,domain_x), legend=:outertopright)
    end

    #plot intersection points
    for i in range(1,stop=length(intersections))
        point = intersections[i]
        color = "blue"
        plot!([point.x],[point.y], seriestype=:scatter, label="", color=color)
    end

    #plot polygons
    for i in range(1,stop=size(polygons)[1])
        xarr = [point.x for point in polygons[i]]
        yarr = [point.y for point in polygons[i]]

        append!(xarr,xarr[1])
        append!(yarr,yarr[1])

        plot!(xarr,yarr,color="magenta", label="")
    end

    #plot circular sectors
    palette = ["red", "magenta", "green", "orange","yellow"]
    for i in 1:length(sectors)
        shade = palette[mod1(i,length(palette))]

        for j in 1:length(sectors[i])
            alpha = (1/j)

            for k in 1:length(sectors[i][j])

                plot!(Functions.draw(sectors[i][j][k][1],sectors[i][j][k][4],sectors[i][j][k][5]), label="", color=shade, linealpha=alpha)

            end
        end

    end

    savefig(string)
end

end #module end