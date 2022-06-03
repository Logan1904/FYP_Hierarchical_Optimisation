module Plotter

export Plot

using Base_Functions
using Plots

function Plot(problem, domain, string)

    circles = problem.circles
    intersections = problem.intersections
    boundaries = problem.boundaries

    plot(minorgrid=true, minorgridalpha=0.25, xlabel="x (m)", ylabel="y (m)")

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
    palette = ["blue", "orange", "green", "red", "purple", "pink", "gray", "olive", "cyan"]
    iter = 1
    for i in 1:length(boundaries)
        for j in 1:length(boundaries[i])
            shade = palette[mod1(iter,length(palette))]
            for k in 1:length(boundaries[i][j])

                if k == length(boundaries[i][j])
                    plot!(draw(boundaries[i][j][k][1],boundaries[i][j][k][4],boundaries[i][j][k][5]), label="C: $i, B: $j", color=shade)
                else
                    plot!(draw(boundaries[i][j][k][1],boundaries[i][j][k][4],boundaries[i][j][k][5]), label="", color=shade)
                end

            end
            
            iter += 1
        end
    end
    savefig(string)
end

end #module end