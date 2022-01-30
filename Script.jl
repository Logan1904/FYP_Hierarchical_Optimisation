import Functions

N_circles = 6
domain_x = 10
domain_y = 10
radius_limit = 2
circles = []

for i in range(1,stop=N_circles)
    x = rand(radius_limit:domain_x-radius_limit)[1]
    y = rand(radius_limit:domain_y-radius_limit)[1]
    R = rand(1:radius_limit)[1]

    push!(circles,Functions.Circle(x,y,R,[],[],false))
end

import Polygonal_Method
import MonteCarlo_Method

Area_Polygonal = Polygonal_Method.Area(circles);
Area_MonteCarlo = MonteCarlo_Method.Area(circles,1000,[domain_x,domain_y]);