module Plotter

export Plot

using Base_Functions
using Plots

function Plot(circles, intersections, boundaries, domain, string)
    p = plot(minorgrid=true, minorgridalpha=0.25)
    domain_x,domain_y = domain

    # plot circles
    for i in range(1,stop=length(circles))
        plot!(draw(circles[i],0.0,2*pi), aspect_ratio=1, label="", color="black", ylim=(0,domain_y), xlim=(0,domain_x), legend=:outertopright)
    end

    # plot intersections
    for i in range(1,stop=length(intersections))
        point = intersections[i]
        color = "blue"
        plot!([point.x],[point.y], seriestype=:scatter, label="", color=color)
    end

    # plot boundaries
    palette = ["red", "magenta", "green", "orange","yellow"]
    for i in 1:length(boundaries)
        shade = palette[mod1(i,length(palette))]

        for j in 1:length(boundaries[i])
            alpha = (1/j)

            for k in 1:length(boundaries[i][j])
                plot!(draw(boundaries[i][j][k][1],boundaries[i][j][k][4],boundaries[i][j][k][5]), label="", color=shade, linealpha=alpha)
            end
        end
    end

    savefig(string)
end

end #module end