using Plots
p = plot()

for i in range(1,stop=length(circles))
    plot!(Functions.draw(circles[i],0,2*pi), aspect_ratio=1, legend=false)
end

for i in range(1,stop=size(intersections)[1])
    point = intersections[i]
    
    if point.ID == 1
        color = "white"
    else
        color = "red"
    end

    plot!([point.x], [point.y], seriestype=:scatter, legend=false, color=color)
end
p