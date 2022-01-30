module Plotter
import Functions
using Plots

function plot_domain(circles,domain,string)
    p = plot()
    domain_x,domain_y = domain
    
    # plot circles
    for i in range(1,stop=length(circles))
        plot!(Functions.draw(circles[i],0,2*pi), aspect_ratio=1, label="", color="black", ylim=(0,domain_y), xlim=(0,domain_x), legend=:outertopright)
    end

    savefig(string)
end

end #module end