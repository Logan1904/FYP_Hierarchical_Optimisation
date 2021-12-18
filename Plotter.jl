using Plots
p = plot()

# plot circles
for i in range(1,stop=length(circles))
    plot!(Functions.draw(circles[i],0,2*pi), aspect_ratio=1, legend=false, color="black")
end

# plot intersection points, white = on boundary contour, red = inside boundary contour
for i in range(1,stop=size(intersections)[1])
    point = intersections[i]
    
    if point.ID == 1
        color = "white"
    else
        color = "red"
    end

    plot!([point.x], [point.y], seriestype=:scatter, legend=false, color=color)
end

# plot polygons
for i in range(1,stop=size(polygon)[1])
    xarr = [point.x for point in polygon[i]]
    yarr = [point.y for point in polygon[i]]

    append!(xarr,xarr[1])
    append!(yarr,yarr[1])

    plot!(xarr,yarr,color="blue")
end

# plot circular sectors
for i in range(1,stop=size(sectors)[1])

    plot!(Functions.draw(sectors[i][1],sectors[i][2],sectors[i][3]), legend=false, colour="red")
end

savefig("image")