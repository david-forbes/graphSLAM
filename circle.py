import math
def circle_gen(x,k,r):
    
        if r**2 - (x-k)**2>=0:
            plusminus = math.sqrt(r**2 - ((x-k)**2))

            return round(k-plusminus), round(k+plusminus) 
        return 0,0

    
