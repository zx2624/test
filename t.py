prefix = 'ADAS_20201106-'
b = ['3.pack,', '2.pack,']
a = [
  1040,
  1050,
  1100,
  1145,
  1200,
  1205,
  1210]
sufix = '08_981_'
for time in a:
  for pack in b:
    print(prefix + str(time) + sufix + pack)
a=[  
  1320,
  1503,
  1533,
  1623  
]
sufix = '19_132_'
for time in a:
  for pack in b:
    print(prefix + str(time) + sufix + pack)
