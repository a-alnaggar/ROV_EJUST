def mapFromTo(num, inMin, inMax, outMin, outMax):
  return  (float(num - inMin) / float(inMax - inMin) * (outMax - outMin)) + outMin

if __name__ == "__main__":
  print(mapFromTo(0, -3,3,1200,1800))
